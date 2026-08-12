"""Unit tests for the pure (no-ROS, no-network) parts of tb3_healthcheck.

Everything here runs without rclpy, without a robot, and without touching the
network, so `colcon test` is meaningful on a laptop with no TB3 in sight.
"""
import importlib.util
import os
import sys

import pytest

_SCRIPT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'scripts', 'tb3_healthcheck.py')


def _load():
    spec = importlib.util.spec_from_file_location('tb3_healthcheck', _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['tb3_healthcheck'] = mod
    spec.loader.exec_module(mod)
    return mod


hc = _load()


# --- phase selection -------------------------------------------------------

def test_phases_are_cumulative():
    assert hc.phases_for('env') == ['env']
    assert hc.phases_for('base') == ['env', 'base']
    assert hc.phases_for('nav') == ['env', 'base', 'nav']


def test_default_phase_is_base_and_excludes_nav_checks():
    args = hc.build_arg_parser().parse_args([])
    assert args.phase == hc.PHASE_BASE
    # The point of the split: map TF / /map / navigate_to_pose must not be
    # part of the preflight you run *before* starting Nav2.
    assert hc.phases_for(args.phase) == ['env', 'base']
    assert hc.PHASE_NAV not in hc.phases_for(args.phase)


def test_all_and_skip_network_normalise():
    parse = hc.build_arg_parser().parse_args
    assert hc.resolve_phase(parse(['--phase', 'all'])) == hc.PHASE_NAV
    assert hc.resolve_phase(parse(['--skip-network'])) == hc.PHASE_ENV
    assert hc.resolve_phase(parse(['--phase', 'nav'])) == hc.PHASE_NAV


def test_nav_checks_are_reported_skipped_not_missing():
    # A base-phase run must still account for every nav check, so a reader
    # can tell "not checked" from "checked and fine".
    assert hc.CHECK_NAMES[hc.PHASE_NAV]


# --- severity / exit codes -------------------------------------------------

def _res(status):
    r = hc.CheckResult('x', hc.PHASE_BASE)
    r.status = status
    return r


def test_status_never_downgrades():
    r = hc.CheckResult('c', hc.PHASE_BASE)
    r.set(hc.BLOCKED, 'bad')
    r.set(hc.WARN, 'less bad')
    r.add('note')
    assert r.status == hc.BLOCKED
    assert len(r.lines) == 3


def test_overall_and_exit_codes():
    assert hc.overall_status([_res(hc.READY), _res(hc.SKIPPED)]) == hc.READY
    assert hc.overall_status([_res(hc.READY), _res(hc.WARN)]) == hc.WARN
    assert hc.overall_status([_res(hc.WARN), _res(hc.BLOCKED)]) == hc.BLOCKED

    assert hc.exit_code_for(hc.READY) == 0
    assert hc.exit_code_for(hc.SKIPPED) == 0
    assert hc.exit_code_for(hc.WARN) == 1
    assert hc.exit_code_for(hc.BLOCKED) == 2
    # WARN can be muted, BLOCKED can not.
    assert hc.exit_code_for(hc.WARN, exit_zero_on_warn=True) == 0
    assert hc.exit_code_for(hc.BLOCKED, exit_zero_on_warn=True) == 2


def test_tool_error_code_does_not_collide_with_warn():
    assert hc.EXIT_TOOL_ERROR not in (hc.EXIT_OK, hc.EXIT_WARN, hc.EXIT_BLOCKED)


# --- FastDDS profile XML ---------------------------------------------------

_PROFILE = """<?xml version="1.0" encoding="UTF-8"?>
<dds{ns}>
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_wide_peers</transport_id>
        <maxInitialPeersRange>{peers}</maxInitialPeersRange>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="unicast_peer"{default}>
      <rtps>
        <useBuiltinTransports>false</useBuiltinTransports>
        <builtin>
          <initialPeersList>
            <locator><udpv4><address>10.32.55.167</address></udpv4></locator>
            <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
          </initialPeersList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
"""

_NS = ' xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"'


def _write_profile(tmp_path, ns=_NS, peers=32, default=' is_default_profile="true"'):
    path = tmp_path / 'fastdds.xml'
    path.write_text(_PROFILE.format(ns=ns, peers=peers, default=default))
    return str(path)


def _run_c1(path, robot_ip=None):
    args = hc.build_arg_parser().parse_args(
        ['--robot-ip', robot_ip] if robot_ip else [])
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = path
    try:
        return hc.check_fastdds_profile(args, None)
    finally:
        os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE', None)


@pytest.mark.parametrize('ns', [_NS, ''])
def test_profile_parsed_with_and_without_namespace(tmp_path, ns):
    # Regression: matching only the namespaced tag made an un-namespaced (but
    # perfectly valid) profile look like every setting was missing.
    res = _run_c1(_write_profile(tmp_path, ns=ns))
    assert res.status == hc.READY, res.lines
    assert any('maxInitialPeersRange=32' in ln for ln in res.lines)
    assert any('useBuiltinTransports=false' in ln for ln in res.lines)
    assert any('10.32.55.167' in ln for ln in res.lines)


def test_profile_without_default_participant_is_blocked(tmp_path):
    res = _run_c1(_write_profile(tmp_path, default=''))
    assert res.status == hc.BLOCKED
    assert any('is_default_profile' in ln for ln in res.lines)


def test_profile_low_peer_range_warns(tmp_path):
    res = _run_c1(_write_profile(tmp_path, peers=4))
    assert res.status == hc.WARN


def test_profile_missing_target_ip_warns(tmp_path):
    res = _run_c1(_write_profile(tmp_path), robot_ip='10.32.55.99')
    assert res.status == hc.WARN
    assert any('not in the initial-peers list' in ln for ln in res.lines)


def test_profile_loopback_is_not_a_peer(tmp_path):
    res = _run_c1(_write_profile(tmp_path))
    assert res.remote_peers == ['10.32.55.167']


def test_bad_xml_is_blocked_not_crash(tmp_path):
    path = tmp_path / 'bad.xml'
    path.write_text('<dds><profiles>')
    res = _run_c1(str(path))
    assert res.status == hc.BLOCKED


def test_missing_profile_is_blocked(tmp_path):
    res = _run_c1(str(tmp_path / 'nope.xml'))
    assert res.status == hc.BLOCKED


# --- robot IP resolution ---------------------------------------------------

def test_robot_ip_precedence(tmp_path, monkeypatch):
    c1 = _run_c1(_write_profile(tmp_path))
    parse = hc.build_arg_parser().parse_args

    monkeypatch.delenv('TB3_ROBOT_IP', raising=False)
    assert hc.resolve_robot_ip(parse([]), [c1])[0] == '10.32.55.167'

    monkeypatch.setenv('TB3_ROBOT_IP', '10.0.0.5')
    assert hc.resolve_robot_ip(parse([]), [c1])[0] == '10.0.0.5'
    assert hc.resolve_robot_ip(parse(['--robot-ip', '1.2.3.4']), [c1])[0] == '1.2.3.4'


def test_loopback_is_detected_as_local():
    assert hc._is_local_address('127.0.0.1')
    assert not hc._is_local_address('10.32.55.167')


# --- battery formatting ----------------------------------------------------

@pytest.mark.parametrize('pct', [float('nan'), None, 0.0])
def test_unreported_percentage_says_so_instead_of_printing_nan(pct):
    # Regression: '%.0f%%' % nan printed 'nan%', and because every NaN
    # comparison is False a NaN reading slid past the voltage guard as
    # "healthy".
    text = hc._percentage_text(pct)
    assert 'not reported' in text
    assert 'nan%' not in text


def test_percentage_scales_both_conventions():
    assert hc._percentage_text(0.87) == 'percentage=87%'
    assert hc._percentage_text(87.0) == 'percentage=87%'


def test_finite_guard():
    assert hc._finite(11.5)
    assert not hc._finite(float('nan'))
    assert not hc._finite(float('inf'))
    assert not hc._finite(None)


# --- rate verdict ----------------------------------------------------------

def test_silent_required_topic_blocks_optional_only_warns():
    r = hc.CheckResult('c', hc.PHASE_BASE)
    hc._rate_verdict(r, '/scan', 0, 3.0, required=True)
    assert r.status == hc.BLOCKED

    r2 = hc.CheckResult('c', hc.PHASE_BASE)
    hc._rate_verdict(r2, '/imu', 0, 3.0, required=False)
    assert r2.status == hc.WARN


def test_rate_uses_measured_elapsed_not_requested_window():
    # 27 messages in 0.9 s is 30 Hz, not 9 Hz. The first version divided by
    # the requested window and under-reported every fast topic.
    r = hc.CheckResult('c', hc.PHASE_BASE)
    hc._rate_verdict(r, '/image_raw/compressed', 27, 0.9, required=True)
    assert r.status == hc.READY
    assert '30.0 Hz' in r.lines[0]


def test_slow_topic_warns():
    r = hc.CheckResult('c', hc.PHASE_BASE)
    hc._rate_verdict(r, '/scan', 6, 3.0, required=True)   # 2 Hz vs nominal 9
    assert r.status == hc.WARN


def test_rate_check_can_be_disabled_for_link_limited_streams():
    # /image_raw runs at ~7 Hz over this WiFi while the compressed stream is
    # healthy at 29 Hz -- that is the link, not a fault.
    r = hc.CheckResult('c', hc.PHASE_BASE)
    hc._rate_verdict(r, '/image_raw', 22, 3.0, required=False, check_rate=False)
    assert r.status == hc.READY
    assert '7.3 Hz' in r.lines[0]


# --- duplicate publishers (orphaned nodes) ---------------------------------

class _FakeEndpoint:
    def __init__(self, name):
        self.node_name = name


class _FakeNode:
    def __init__(self, mapping):
        self.mapping = mapping

    def get_publishers_info_by_topic(self, topic):
        return [_FakeEndpoint(n) for n in self.mapping.get(topic, [])]


def test_duplicate_publisher_warns():
    # Observed live: two ld08_driver processes, one orphaned from an earlier
    # bringup, both on /scan.
    r = hc.CheckResult('c', hc.PHASE_BASE)
    node = _FakeNode({'/scan': ['ld08_driver', 'ld08_driver']})
    hc._warn_on_duplicate_publishers(r, node, '/scan', 'orphan')
    assert r.status == hc.WARN
    assert 'has 2 publishers' in r.lines[0]


def test_single_publisher_is_quiet():
    r = hc.CheckResult('c', hc.PHASE_BASE)
    hc._warn_on_duplicate_publishers(
        r, _FakeNode({'/scan': ['ld08_driver']}), '/scan', 'orphan')
    assert r.status == hc.READY
    assert r.lines == []


# --- ping RTT parsing ------------------------------------------------------

_PING_OUT = """PING 10.32.55.167 (10.32.55.167) 56(84) bytes of data.
64 bytes from 10.32.55.167: icmp_seq=1 ttl=64 time=61.7 ms
64 bytes from 10.32.55.167: icmp_seq=2 ttl=64 time=83.7 ms

--- 10.32.55.167 ping statistics ---
2 packets transmitted, 2 received, 0% packet loss, time 1002ms
rtt min/avg/max/mdev = 61.702/72.701/83.700/10.999 ms
"""


def test_ping_rtt_uses_average_not_first_sample():
    assert hc._parse_ping_rtt(_PING_OUT) == pytest.approx(72.701)


def test_ping_rtt_falls_back_to_per_packet_time():
    partial = 'PING x\n64 bytes from x: icmp_seq=1 ttl=64 time=99.5 ms\n'
    assert hc._parse_ping_rtt(partial) == pytest.approx(99.5)


def test_ping_rtt_none_when_unparseable():
    assert hc._parse_ping_rtt('100% packet loss') is None


# --- /scan content sanity --------------------------------------------------

class _FakeScan:
    def __init__(self, ranges):
        self.ranges = ranges
        self.range_min = 0.12
        self.range_max = 3.5


def test_all_inf_scan_warns():
    r = hc.CheckResult('c', hc.PHASE_BASE)
    hc._scan_sanity(r, _FakeScan([float('inf')] * 360), 0.10)
    assert r.status == hc.WARN


def test_normal_scan_ok():
    r = hc.CheckResult('c', hc.PHASE_BASE)
    ranges = [1.0] * 180 + [float('inf')] * 180
    hc._scan_sanity(r, _FakeScan(ranges), 0.10)
    assert r.status == hc.READY


def test_empty_scan_blocks():
    r = hc.CheckResult('c', hc.PHASE_BASE)
    hc._scan_sanity(r, _FakeScan([]), 0.10)
    assert r.status == hc.BLOCKED


# --- reporting -------------------------------------------------------------

def test_text_report_lists_blockers_and_exit_code():
    results = [_res(hc.READY), _res(hc.BLOCKED)]
    results[1].name = 'C3 base topics + rates'
    text = hc.render_text(results, hc.BLOCKED, hc.PHASE_BASE, 2)
    assert 'NOT READY' in text
    assert 'C3 base topics + rates' in text
    assert 'exit 2' in text


def test_env_phase_needs_no_ros_and_no_network(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '2')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('TURTLEBOT3_MODEL', 'burger')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE',
                       _write_profile(tmp_path))
    args = hc.build_arg_parser().parse_args(['--phase', 'env'])
    results, active = hc.run_checks(args)
    assert active == ['env']
    statuses = {r.name: r.status for r in results}
    # Every non-env check accounted for as SKIPPED, nothing silently absent.
    for name in hc.CHECK_NAMES[hc.PHASE_BASE] + hc.CHECK_NAMES[hc.PHASE_NAV]:
        assert statuses[name] == hc.SKIPPED
    assert statuses['C2 robot reachability / SSH port'] == hc.SKIPPED


def test_domain_mismatch_blocks(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('TURTLEBOT3_MODEL', 'burger')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', __file__)
    args = hc.build_arg_parser().parse_args([])
    res = hc.check_host_env(args, None)
    assert res.status == hc.BLOCKED
    assert any('never see each other' in ln for ln in res.lines)


def test_localhost_only_blocks(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '2')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('TURTLEBOT3_MODEL', 'burger')
    monkeypatch.setenv('FASTRTPS_DEFAULT_PROFILES_FILE', __file__)
    res = hc.check_host_env(hc.build_arg_parser().parse_args([]), None)
    assert res.status == hc.BLOCKED
