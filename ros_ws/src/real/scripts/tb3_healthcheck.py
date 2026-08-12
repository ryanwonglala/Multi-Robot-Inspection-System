#!/usr/bin/env python3
"""Read-only preflight/healthcheck for the physical TB3 (laptop side).

Never publishes cmd_vel, never sends a Nav2 goal, never writes to the robot.
Every check either proves the corresponding piece is reachable/healthy or
reports it as BLOCKED with the reason -- there is no default-READY fallback,
because a false "READY" on a preflight tool is worse than a false BLOCKED.

Phases (each is a superset of the previous one; pick with --phase):

    env    C0 host environment  - env vars required for campus-WiFi DDS
           C1 FastDDS profile   - initial-peers XML sanity (P1 lessons)

    base   (default) everything needed for, and produced by, the robot-side
           bringup -- i.e. what must be true BEFORE you start Nav2:
           C2 robot reachability  - ICMP ping + connect-only TCP probe of :22
           C3 base topics         - /scan /odom /battery_state /imu + rates
           C4 odom TF             - odom->base_footprint, base_footprint->base_scan
           C5 battery             - voltage/percentage sanity
           C6 camera              - /image_raw(/compressed) + /camera_info K

    nav    additionally what only exists once Nav2/AMCL is up:
           C7 map TF              - map->odom, map->base_footprint
           C8 /map                - latched OccupancyGrid actually received
           C9 Nav2 action         - navigate_to_pose (wait_for_server only)

The split matters: map->base_footprint and navigate_to_pose cannot possibly
exist before Nav2 is launched, so requiring them in the default preflight
turns "ready to launch Nav2" into an unsatisfiable loop.

QoS: every monitored sensor topic is subscribed with BEST_EFFORT/VOLATILE
(qos_profile_sensor_data). A RELIABLE subscriber is *incompatible* with the
BEST_EFFORT publishers used by the LDS driver and v4l2_camera, and the only
symptom is zero messages -- i.e. the tool would report a healthy lidar as
"0 Hz / BLOCKED". BEST_EFFORT subscribers match both publisher kinds, so a
best-effort monitor can never manufacture that false negative. /map is the
one exception: it is latched, so it needs RELIABLE/TRANSIENT_LOCAL.

Exit code: 0 all READY/SKIPPED, 1 any WARN, 2 any BLOCKED, 3 tool error
(bad env, missing rclpy, unexpected exception), 130 interrupted.
"""
import argparse
import json
import math
import os
import socket
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

READY = 'READY'
WARN = 'WARN'
BLOCKED = 'BLOCKED'
SKIPPED = 'SKIPPED'

_SEVERITY = {READY: 0, SKIPPED: 0, WARN: 1, BLOCKED: 2}

EXIT_OK = 0
EXIT_WARN = 1
EXIT_BLOCKED = 2
EXIT_TOOL_ERROR = 3
EXIT_INTERRUPTED = 130

PHASE_ENV = 'env'
PHASE_BASE = 'base'
PHASE_NAV = 'nav'
PHASE_ORDER = [PHASE_ENV, PHASE_BASE, PHASE_NAV]

# Nominal rates observed and recorded in the P1 smoke test (LDS-02 ~9 Hz,
# odom ~20 Hz, camera ~30 fps, OpenCR battery/imu on the sensor-state loop).
# Warn threshold is a generous fraction of nominal so WiFi jitter doesn't
# cry wolf; the campus link runs ~70 ms RTT and drops frames under load.
NOMINAL_HZ = {
    '/scan': 9.0,
    '/odom': 20.0,
    # turtlebot3_node publishes imu and battery_state off the same ~20 Hz
    # sensor-state loop on this bringup, not at the IMU's raw rate.
    '/imu': 20.0,
    '/battery_state': 5.0,
    '/image_raw/compressed': 15.0,
    # /image_raw (uncompressed 640x480 rgb8) cannot sustain 30 fps over this
    # WiFi link and is only rate-judged when the compressed stream is absent.
    '/image_raw': 15.0,
}
WARN_HZ_FRACTION = 0.5

DEFAULT_FASTDDS_PROFILE = os.path.expanduser('~/fastdds_laptop.xml')
MIN_INITIAL_PEERS_RANGE = 32
EXPECTED_DOMAIN_ID = 2
EXPECTED_RMW = 'rmw_fastrtps_cpp'
EXPECTED_MODEL = 'burger'

# 3S LiPo on the Burger: OpenCR starts its low-voltage alarm around 10.5 V and
# motor torque/stopping precision degrades well before that.
WARN_VOLTAGE = 11.0
BLOCK_VOLTAGE = 10.5


class CheckResult:
    def __init__(self, name, phase):
        self.name = name
        self.phase = phase
        self.status = READY
        self.lines = []
        # Filled in by C1: initial-peer addresses that are not this laptop.
        self.remote_peers = []

    def add(self, line):
        self.lines.append(line)

    def set(self, status, line):
        if _SEVERITY[status] > _SEVERITY[self.status]:
            self.status = status
        self.add(line)

    def skip(self, reason):
        self.status = SKIPPED
        self.add(reason)
        return self

    def to_dict(self):
        return {
            'check': self.name,
            'phase': self.phase,
            'status': self.status,
            'details': self.lines,
        }


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------

def _local_tag(elem):
    """Tag name with any XML namespace stripped."""
    tag = elem.tag
    if not isinstance(tag, str):
        return ''
    return tag.rsplit('}', 1)[-1]


def _iter_local(root, name):
    """Yield elements whose local tag is `name`, namespace-agnostic.

    FastDDS accepts the profile XML both with and without the eprosima
    namespace declaration. Matching on the namespaced tag only (as the first
    version did) silently found nothing on an un-namespaced profile and then
    reported every sanity check as "not set".
    """
    for elem in root.iter():
        if _local_tag(elem) == name:
            yield elem


def _is_local_address(ip):
    """True if `ip` belongs to this host (so it is not the robot)."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.bind((ip, 0))
        return True
    except OSError:
        return False
    finally:
        probe.close()


def _finite(value):
    try:
        return value is not None and math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _percentage_text(pct):
    """Human text for BatteryState.percentage, which the OpenCR often leaves
    unset (NaN or 0.0). Formatting NaN as '%.0f%%' printed 'nan%' and, worse,
    every NaN comparison is False so a NaN reading slipped through the
    low-voltage guard as if it were healthy."""
    if not _finite(pct) or float(pct) == 0.0:
        return 'percentage not reported by OpenCR (%r) -- judge by voltage' % pct
    pct = float(pct)
    if pct <= 1.0:
        return 'percentage=%.0f%%' % (pct * 100.0)
    return 'percentage=%.0f%%' % pct


def _policy_name(policy):
    return getattr(policy, 'name', str(policy))


def _endpoint_name(endpoint):
    """rmw_fastrtps reports '_NODE_NAME_UNKNOWN_' for a participant whose node
    info has not been discovered yet; say that plainly instead."""
    name = getattr(endpoint, 'node_name', '') or ''
    if not name or 'UNKNOWN' in name:
        return '<node name not yet discovered>'
    return name


def _publisher_summary(node, topic):
    """'2 publisher(s): turtlebot3_node[BEST_EFFORT/VOLATILE], ...' or None.

    Read-only graph introspection: shows the *publisher's* QoS, which is how
    you tell a genuinely dead topic from a QoS-incompatible subscriber.
    """
    try:
        endpoints = node.get_publishers_info_by_topic(topic)
    except Exception:
        return None
    if not endpoints:
        return None
    parts = []
    for ep in endpoints:
        qos = getattr(ep, 'qos_profile', None)
        if qos is None:
            parts.append(_endpoint_name(ep))
        else:
            parts.append('%s[%s/%s]' % (_endpoint_name(ep),
                                        _policy_name(qos.reliability),
                                        _policy_name(qos.durability)))
    return '%d publisher(s): %s' % (len(endpoints), ', '.join(parts))


class RosCtx:
    """Lazily-created rclpy node + TF buffer shared by the ROS checks."""

    def __init__(self):
        self.rclpy = None
        self.node = None
        self._tf_buffer = None
        self._tf_listener = None

    def start(self):
        import rclpy
        from rclpy.node import Node
        rclpy.init(args=[])
        self.rclpy = rclpy
        self.node = Node('tb3_healthcheck')

    def stop(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.rclpy is not None:
            try:
                if self.rclpy.ok():
                    self.rclpy.shutdown()
            except Exception:
                pass
            self.rclpy = None

    def spin_for(self, seconds):
        """Spin for a wall-clock duration.

        spin_once() returns as soon as one callback fires, so the first
        version's `for _ in range(int(window / 0.1)): spin_once(0.1)` finished
        in a fraction of `window` whenever messages were flowing -- it then
        divided the message count by the *intended* window and under-reported
        every rate (a 30 fps camera came out as 10 Hz). Always bound loops by
        the monotonic clock.
        """
        deadline = time.monotonic() + seconds
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return
            self.rclpy.spin_once(self.node, timeout_sec=min(remaining, 0.05))

    def tf_buffer(self):
        if self._tf_buffer is None:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            # Keep a reference: the listener owns the /tf + /tf_static
            # subscriptions and dropping it would stop the buffer filling.
            self._tf_listener = tf2_ros.TransformListener(
                self._tf_buffer, self.node, spin_thread=False)
        return self._tf_buffer

    def wait_for_topics(self, names, timeout):
        """Poll the graph until every name in `names` appears, or timeout.

        Returns the final {topic: [types]} mapping. Polling beats a single
        sleep-then-snapshot: unicast discovery over campus WiFi is usually
        fast but occasionally needs seconds, and a fixed wait pays the worst
        case every run.
        """
        wanted = set(names)
        deadline = time.monotonic() + timeout
        known = {}
        while True:
            known = dict(self.node.get_topic_names_and_types())
            if wanted.issubset(known):
                return known
            if time.monotonic() >= deadline:
                return known
            self.spin_for(0.2)

    def sample_topic(self, msg_cls, topic, window, qos):
        """Subscribe for `window` seconds; return (count, elapsed, last_msg)."""
        state = {'count': 0, 'last': None}

        def _cb(msg):
            state['count'] += 1
            state['last'] = msg

        sub = self.node.create_subscription(msg_cls, topic, _cb, qos)
        started = time.monotonic()
        try:
            self.spin_for(window)
        finally:
            elapsed = time.monotonic() - started
            self.node.destroy_subscription(sub)
        return state['count'], max(elapsed, 1e-6), state['last']

    def wait_for_message(self, msg_cls, topic, timeout, qos):
        """Return the first message on `topic` within `timeout`, else None."""
        state = {'msg': None}

        def _cb(msg):
            state['msg'] = msg

        sub = self.node.create_subscription(msg_cls, topic, _cb, qos)
        deadline = time.monotonic() + timeout
        try:
            while state['msg'] is None and time.monotonic() < deadline:
                remaining = deadline - time.monotonic()
                self.rclpy.spin_once(self.node,
                                     timeout_sec=min(max(remaining, 0.0), 0.05))
        finally:
            self.node.destroy_subscription(sub)
        return state['msg']


def _sensor_qos():
    """BEST_EFFORT/VOLATILE/KeepLast(5): compatible with every publisher we
    monitor, including the RELIABLE ones (a best-effort reader matches a
    reliable writer; the reverse does not match)."""
    from rclpy.qos import qos_profile_sensor_data
    return qos_profile_sensor_data


def _latched_qos(depth=1):
    """RELIABLE/TRANSIENT_LOCAL: required to receive an already-published
    latched topic such as /map or /robot_description."""
    from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                           QoSDurabilityPolicy, QoSHistoryPolicy)
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def _warn_on_duplicate_publishers(result, node, topic, hint):
    """One driver per sensor topic. Two means an orphan survived a previous
    bringup and both copies are fighting over the same device, which shows up
    as a halved rate and corrupted messages rather than as an error."""
    try:
        endpoints = node.get_publishers_info_by_topic(topic)
    except Exception:
        return
    if len(endpoints) > 1:
        result.set(WARN, '%s has %d publishers (%s) -- %s'
                   % (topic, len(endpoints),
                      ', '.join(_endpoint_name(ep) for ep in endpoints),
                      hint))


_ORPHAN_HINT = ('almost certainly an orphaned node from an earlier bringup '
                '(reparented to init, so the launch-file pkill missed it); '
                'two drivers on one device halve the rate and interleave '
                'garbage')


def _rate_verdict(result, topic, count, elapsed, required, check_rate=True):
    """Shared rate judgement. `required` topics BLOCK when silent, optional
    ones only WARN. `check_rate=False` reports the measured Hz without
    judging it (for streams whose rate is legitimately link-limited)."""
    if count == 0:
        msg = ('%s: publisher advertised but 0 messages in %.1fs '
               '(subscribed BEST_EFFORT, so this is not a QoS mismatch on '
               'our side -- check the driver on the robot)' % (topic, elapsed))
        result.set(BLOCKED if required else WARN, msg)
        return
    hz = count / elapsed
    nominal = NOMINAL_HZ.get(topic) if check_rate else None
    text = '%s: %.1f Hz (%d msgs / %.1fs)' % (topic, hz, count, elapsed)
    if nominal and hz < nominal * WARN_HZ_FRACTION:
        result.set(WARN, '%s, below %.0f%% of nominal %.0f Hz'
                   % (text, WARN_HZ_FRACTION * 100, nominal))
    else:
        result.add(text)


# --------------------------------------------------------------------------
# C0 / C1 -- env phase (no network, no ROS)
# --------------------------------------------------------------------------

def check_host_env(args, ctx):
    r = CheckResult('C0 host environment', PHASE_ENV)
    domain = os.environ.get('ROS_DOMAIN_ID')
    rmw = os.environ.get('RMW_IMPLEMENTATION')
    profile = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
    localhost_only = os.environ.get('ROS_LOCALHOST_ONLY')
    model = os.environ.get('TURTLEBOT3_MODEL')

    if args.expect_domain < 0:
        r.add('ROS_DOMAIN_ID=%s (not checked, --expect-domain -1)' % domain)
    elif domain is None:
        r.set(BLOCKED, 'ROS_DOMAIN_ID not set -- defaults to 0 while the '
                       'robot publishes on %d, so the graph will look empty. '
                       'source env_real.sh.' % args.expect_domain)
    elif domain.strip() != str(args.expect_domain):
        r.set(BLOCKED, 'ROS_DOMAIN_ID=%s but the robot bringup uses %d -- '
                       'different domains never see each other'
                       % (domain, args.expect_domain))
    else:
        r.add('ROS_DOMAIN_ID=%s' % domain)

    if rmw != EXPECTED_RMW:
        r.set(BLOCKED, 'RMW_IMPLEMENTATION=%r (expected %s). Campus WiFi '
                       'blocks multicast discovery; any other RMW ignores the '
                       'unicast initial-peers profile and the robot will '
                       'never be seen.' % (rmw, EXPECTED_RMW))
    else:
        r.add('RMW_IMPLEMENTATION=%s' % EXPECTED_RMW)

    if localhost_only == '1':
        r.set(BLOCKED, 'ROS_LOCALHOST_ONLY=1 -- this laptop cannot see any '
                       'off-host participant, including the robot')
    elif localhost_only is not None:
        r.add('ROS_LOCALHOST_ONLY=%s' % localhost_only)

    if not profile:
        r.set(BLOCKED, 'FASTRTPS_DEFAULT_PROFILES_FILE not set -- without it '
                       'FastDDS falls back to multicast discovery, which '
                       'campus WiFi drops silently')
    elif not os.path.isfile(profile):
        r.set(BLOCKED, 'FASTRTPS_DEFAULT_PROFILES_FILE=%s does not exist'
              % profile)
    else:
        r.add('FASTRTPS_DEFAULT_PROFILES_FILE=%s' % profile)

    if model != EXPECTED_MODEL:
        r.set(WARN, 'TURTLEBOT3_MODEL=%r (expected %s)' % (model, EXPECTED_MODEL))
    else:
        r.add('TURTLEBOT3_MODEL=%s' % EXPECTED_MODEL)

    if subprocess.run(['which', 'ros2'], stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL).returncode != 0:
        r.set(BLOCKED, 'ros2 CLI not found on PATH -- source env_real.sh first')
    else:
        r.add('ros2 CLI on PATH')

    return r


def check_fastdds_profile(args, ctx):
    r = CheckResult('C1 FastDDS profile', PHASE_ENV)
    path = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE') or DEFAULT_FASTDDS_PROFILE
    if not os.path.isfile(path):
        r.set(BLOCKED, 'profile file not found: %s' % path)
        return r

    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        r.set(BLOCKED, 'profile file is not valid XML: %s' % exc)
        return r

    r.add('parsed %s' % path)

    # A participant profile that is not the default profile is loaded but
    # never applied unless a node asks for it by name -- silently leaving
    # discovery on multicast.
    participants = list(_iter_local(root, 'participant'))
    if not participants:
        r.set(BLOCKED, 'no <participant> profile in the XML -- nothing will '
                       'apply the initial-peers list')
    elif not any(p.get('is_default_profile', '').lower() == 'true'
                 for p in participants):
        r.set(BLOCKED, 'no <participant ... is_default_profile="true"> -- the '
                       'profile parses but FastDDS will not apply it, so '
                       'discovery silently stays on multicast')
    else:
        r.add('default participant profile present')

    ranges = [int(e.text.strip()) for e in _iter_local(root, 'maxInitialPeersRange')
              if e.text and e.text.strip().isdigit()]
    if not ranges:
        r.set(WARN, 'maxInitialPeersRange not found (default is 4 -- too low '
                    'once rviz/cartographer/CLIs are all running)')
    elif min(ranges) < MIN_INITIAL_PEERS_RANGE:
        r.set(WARN, 'maxInitialPeersRange=%d (< %d recommended per P1 '
                    'incident notes)' % (min(ranges), MIN_INITIAL_PEERS_RANGE))
    else:
        r.add('maxInitialPeersRange=%d' % min(ranges))

    builtin = [e.text.strip().lower() for e in _iter_local(root, 'useBuiltinTransports')
               if e.text]
    if not builtin:
        r.set(WARN, 'useBuiltinTransports not set explicitly (expected false)')
    elif any(v != 'false' for v in builtin):
        r.set(WARN, 'useBuiltinTransports=true somewhere in profile (expected false)')
    else:
        r.add('useBuiltinTransports=false')

    peers = [e.text.strip() for e in _iter_local(root, 'address')
             if e.text and e.text.strip() not in ('127.0.0.1', 'localhost')]
    if not peers:
        r.set(WARN, 'no non-loopback initial peer address found in profile')
    else:
        r.add('initial peer address(es): %s' % ', '.join(peers))
        if args.robot_ip and args.robot_ip not in peers:
            r.set(WARN, 'target robot %s is not in the initial-peers list -- '
                        'discovery will not reach it' % args.robot_ip)

    r.remote_peers = [p for p in peers if not _is_local_address(p)]
    if peers and not r.remote_peers:
        r.set(WARN, 'every initial peer address is local to this laptop -- '
                    'the robot address is missing from the profile')
    return r


# --------------------------------------------------------------------------
# C2 -- reachability
# --------------------------------------------------------------------------

def resolve_robot_ip(args, env_results):
    if args.robot_ip:
        return args.robot_ip, 'command line'
    from_env = os.environ.get('TB3_ROBOT_IP')
    if from_env:
        return from_env.strip(), 'TB3_ROBOT_IP'
    for res in env_results:
        if res.remote_peers:
            return res.remote_peers[0], 'FastDDS profile initial peer'
    return None, None


def check_robot_reachability(args, ctx):
    r = CheckResult('C2 robot reachability / SSH port', PHASE_BASE)
    ip = args.resolved_robot_ip
    if not ip:
        r.set(BLOCKED, 'robot IP unknown -- pass --robot-ip, set '
                       'TB3_ROBOT_IP, or fix the FastDDS initial peer')
        return r
    r.add('target=%s (from %s)' % (ip, args.resolved_robot_ip_source))

    # -W wants whole seconds on older iputils; ceil so a 1.5s request never
    # rounds down to an instant timeout. subprocess timeout is a backstop in
    # case ping itself wedges (it has hung on this WiFi during roaming).
    wait_s = max(1, int(math.ceil(args.ping_timeout)))
    try:
        # 3 packets, averaged: a single sample on this WiFi swings between
        # 60 ms and 200 ms and would flip the RTT warning at random.
        ping = subprocess.run(
            ['ping', '-n', '-c', str(args.ping_count), '-i', '0.3',
             '-W', str(wait_s), ip],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=wait_s + 2 + args.ping_count)
    except FileNotFoundError:
        r.set(WARN, 'ping binary not found, cannot verify ICMP reachability')
        ping = None
    except subprocess.TimeoutExpired:
        r.set(BLOCKED, 'ping to %s did not return within %ds'
              % (ip, wait_s + 2 + args.ping_count))
        ping = None

    if ping is not None:
        if ping.returncode == 0:
            rtt = _parse_ping_rtt(ping.stdout.decode('utf-8', 'replace'))
            r.add('ping OK%s' % (' (%.0f ms avg RTT over %d)'
                                 % (rtt, args.ping_count) if rtt else ''))
            if rtt and rtt > args.rtt_warn_ms:
                r.set(WARN, 'RTT %.0f ms > %.0f ms -- expect laggy image '
                            'transport and slow action discovery'
                            % (rtt, args.rtt_warn_ms))
        else:
            r.set(BLOCKED, 'ping to %s failed (no ICMP reply within %ds). '
                           'Note: this network never lists the robot in the '
                           'laptop ARP table even when it answers pings -- do '
                           'not use arp-scan as a substitute.' % (ip, wait_s))

    # Connect-only TCP probe: no auth attempt, no ssh binary, nothing written.
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(args.ssh_timeout)
    try:
        errno_ = sock.connect_ex((ip, 22))
        if errno_ == 0:
            r.add('SSH port 22 open (probe only, no login attempted)')
        else:
            r.set(WARN, 'SSH port 22 not reachable (errno %d) -- remote '
                        'deploy of tb3_robot_start.sh would not work, but '
                        'this alone does not block a bringup that is already '
                        'running' % errno_)
    except socket.error as exc:
        r.set(WARN, 'SSH port probe failed: %s' % exc)
    finally:
        sock.close()

    return r


def _parse_ping_rtt(text):
    """Average RTT in ms from ping's summary line, falling back to the first
    per-packet 'time=' if the summary is absent (e.g. partial packet loss)."""
    for line in text.splitlines():
        if 'min/avg/max' in line and '=' in line:
            fields = line.split('=', 1)[1].strip().split()[0].split('/')
            if len(fields) >= 2:
                try:
                    return float(fields[1])
                except ValueError:
                    pass
    for token in text.split():
        if token.startswith('time='):
            try:
                return float(token[5:])
            except ValueError:
                return None
    return None


# --------------------------------------------------------------------------
# C3 / C4 / C5 / C6 -- base phase
# --------------------------------------------------------------------------

def check_base_topics(args, ctx):
    r = CheckResult('C3 base topics + rates', PHASE_BASE)
    from sensor_msgs.msg import LaserScan, BatteryState, Imu
    from nav_msgs.msg import Odometry

    specs = [
        ('/scan', 'sensor_msgs/msg/LaserScan', LaserScan, True),
        ('/odom', 'nav_msgs/msg/Odometry', Odometry, True),
        ('/battery_state', 'sensor_msgs/msg/BatteryState', BatteryState, True),
        ('/imu', 'sensor_msgs/msg/Imu', Imu, False),
    ]

    known = ctx.wait_for_topics([s[0] for s in specs], args.discovery_wait)
    qos = _sensor_qos()

    for topic, type_str, msg_cls, required in specs:
        types = known.get(topic)
        if not types:
            r.set(BLOCKED if required else WARN,
                  '%s: not seen in ROS graph (no publisher after %.1fs '
                  'discovery) -- robot bringup not running, or DDS/domain '
                  'mismatch' % (topic, args.discovery_wait))
            continue
        if type_str not in types:
            r.set(WARN, '%s: publisher present but unexpected type(s) %s'
                  % (topic, types))
            continue

        pubs = _publisher_summary(ctx.node, topic)
        if pubs:
            r.add('%s advertised by %s' % (topic, pubs))
        _warn_on_duplicate_publishers(r, ctx.node, topic, _ORPHAN_HINT)

        count, elapsed, last = ctx.sample_topic(msg_cls, topic,
                                                args.rate_window, qos)
        _rate_verdict(r, topic, count, elapsed, required)

        if topic == '/scan' and last is not None:
            _scan_sanity(r, last, args.min_scan_valid_fraction)

    return r


def _scan_sanity(result, scan, min_valid_fraction):
    """A lidar can publish at full rate and still be useless (all-inf returns
    when the cover is dusty or the belt is off), which Nav2 turns into a
    silent no-obstacle world."""
    finite = [v for v in scan.ranges
              if _finite(v) and scan.range_min <= v <= scan.range_max]
    total = len(scan.ranges)
    if total == 0:
        result.set(BLOCKED, '/scan: empty ranges array')
        return
    frac = len(finite) / float(total)
    text = ('/scan: %d/%d valid returns (%.0f%%), range %.2f-%.2f m'
            % (len(finite), total, frac * 100, scan.range_min, scan.range_max))
    if frac < min_valid_fraction:
        result.set(WARN, text + ' -- mostly inf/NaN; check the LDS window and '
                                'that the robot is not in open space')
    else:
        result.add(text)


def check_odom_tf(args, ctx):
    r = CheckResult('C4 odom TF', PHASE_BASE)
    buf = ctx.tf_buffer()
    # base_footprint->base_scan is static (robot_state_publisher); without it
    # Nav2 cannot place /scan in the costmap even though /scan is healthy.
    pairs = [('odom', 'base_footprint'), ('base_footprint', 'base_scan')]
    _check_tf_pairs(r, args, ctx, buf, pairs, args.tf_timeout,
                    'robot bringup / robot_state_publisher')

    # An orphaned robot_state_publisher from a previous bringup keeps
    # publishing the same /tf_static, which shows up as TF_REPEATED_DATA spam
    # and two competing /robot_description latches.
    for topic in ('/robot_description', '/tf_static'):
        _warn_on_duplicate_publishers(
            r, ctx.node, topic,
            'orphaned robot_state_publisher from an earlier bringup; expect '
            'TF_REPEATED_DATA warnings and a stale /robot_description in RViz')
    return r


def _check_tf_pairs(result, args, ctx, buf, pairs, timeout, blame):
    from rclpy.time import Time
    deadline = time.monotonic() + timeout
    pending = list(pairs)
    while pending and time.monotonic() < deadline:
        ctx.spin_for(0.1)
        still = []
        for target, source in pending:
            if buf.can_transform(target, source, Time()):
                result.add('%s -> %s available' % (target, source))
            else:
                still.append((target, source))
        pending = still
    for target, source in pending:
        result.set(BLOCKED, '%s -> %s not available within %.1fs (%s not '
                            'running?)' % (target, source, timeout, blame))
    if pending:
        frames = buf.all_frames_as_string().replace('\n', ' | ').strip()
        result.add('frames currently in the TF buffer: %s'
                   % (frames[:400] if frames else '(none)'))


def check_battery(args, ctx):
    r = CheckResult('C5 battery', PHASE_BASE)
    from sensor_msgs.msg import BatteryState

    msg = ctx.wait_for_message(BatteryState, '/battery_state',
                               args.battery_timeout, _sensor_qos())
    if msg is None:
        r.set(BLOCKED, 'no /battery_state message within %.1fs -- OpenCR not '
                       'talking to the LattePanda (check its power switch and '
                       'the USB cable)' % args.battery_timeout)
        return r

    voltage = msg.voltage
    r.add(_percentage_text(msg.percentage))

    if not _finite(voltage):
        r.set(WARN, 'voltage is %r (non-finite) -- OpenCR is publishing but '
                    'not reporting a usable reading; the low-voltage guard '
                    'cannot protect you' % voltage)
        return r

    if voltage <= 0.01:
        r.set(WARN, 'voltage=%.2fV -- reads as zero, treat as "no reading" '
                    'rather than as an empty pack' % voltage)
    elif voltage < args.block_voltage:
        r.set(BLOCKED, 'voltage %.2fV < %.2fV -- OpenCR low-voltage alarm '
                       'territory; charge before driving'
              % (voltage, args.block_voltage))
    elif voltage < args.warn_voltage:
        r.set(WARN, 'voltage %.2fV < %.2fV -- known to degrade docking/'
                    'stopping precision' % (voltage, args.warn_voltage))
    else:
        r.add('voltage=%.2fV' % voltage)

    return r


def check_camera(args, ctx):
    r = CheckResult('C6 camera', PHASE_BASE)
    from sensor_msgs.msg import Image, CameraInfo
    from sensor_msgs.msg import CompressedImage

    wanted = ['/image_raw', '/image_raw/compressed', '/camera_info']
    known = ctx.wait_for_topics(wanted, args.discovery_wait)
    qos = _sensor_qos()

    has_raw = '/image_raw' in known
    has_comp = '/image_raw/compressed' in known
    if not has_raw and not has_comp:
        r.set(BLOCKED, 'neither /image_raw nor /image_raw/compressed seen in '
                       'the ROS graph -- v4l2_camera_node not running. The '
                       'photo-diff inspection has no input without it.')
        return r

    # Prefer the compressed stream for the rate probe: it is what actually
    # crosses the WiFi link in a live run.
    if has_comp:
        pubs = _publisher_summary(ctx.node, '/image_raw/compressed')
        if pubs:
            r.add('/image_raw/compressed advertised by %s' % pubs)
        count, elapsed, _ = ctx.sample_topic(CompressedImage,
                                             '/image_raw/compressed',
                                             args.rate_window, qos)
        _rate_verdict(r, '/image_raw/compressed', count, elapsed, True)
    if has_raw:
        _warn_on_duplicate_publishers(r, ctx.node, '/image_raw', _ORPHAN_HINT)
        count, elapsed, _ = ctx.sample_topic(Image, '/image_raw',
                                             args.rate_window, qos)
        # When the compressed stream is up it is the one that matters; the raw
        # topic is throttled by the WiFi link, not by a fault.
        _rate_verdict(r, '/image_raw', count, elapsed, not has_comp,
                      check_rate=not has_comp)
    elif has_comp:
        r.add('/image_raw not published (compressed only -- expected, the '
              'raw stream is too heavy for this WiFi link)')

    if '/camera_info' not in known:
        r.set(WARN, '/camera_info not seen -- inspection_runner will fall '
                    'back to default intrinsics')
        return r

    info = ctx.wait_for_message(CameraInfo, '/camera_info',
                                args.rate_window, qos)
    if info is None:
        r.set(WARN, '/camera_info advertised but no message within %.1fs'
              % args.rate_window)
        return r

    fx, fy = info.k[0], info.k[4]
    if not _finite(fx) or not _finite(fy) or fx <= 0.0 or fy <= 0.0:
        r.set(WARN, 'camera_info K has fx=%r fy=%r (uncalibrated -- '
                    'v4l2_camera found no calibration file) so the runner '
                    'guard will substitute default intrinsics' % (fx, fy))
    else:
        r.add('camera_info K looks calibrated (fx=%.1f fy=%.1f, %dx%d)'
              % (fx, fy, info.width, info.height))
    return r


# --------------------------------------------------------------------------
# C7 / C8 / C9 -- nav phase
# --------------------------------------------------------------------------

def check_map_tf(args, ctx):
    r = CheckResult('C7 map TF (localization)', PHASE_NAV)
    buf = ctx.tf_buffer()
    _check_tf_pairs(r, args, ctx, buf,
                    [('map', 'odom'), ('map', 'base_footprint')],
                    args.tf_timeout,
                    'AMCL/localization inside nav_real.launch.py')
    return r


def check_map_topic(args, ctx):
    r = CheckResult('C8 /map', PHASE_NAV)
    from nav_msgs.msg import OccupancyGrid

    # /map is latched (RELIABLE + TRANSIENT_LOCAL). A sensor-data QoS
    # subscriber would connect and then wait forever, because the one and only
    # publication happened before we subscribed.
    grid = ctx.wait_for_message(OccupancyGrid, '/map', args.map_timeout,
                                _latched_qos())
    if grid is None:
        pubs = _publisher_summary(ctx.node, '/map')
        r.set(BLOCKED, 'no latched /map within %.1fs (%s) -- map_server not '
                       'up or not activated' % (args.map_timeout,
                                                pubs or 'no publisher seen'))
        return r
    info = grid.info
    r.add('/map %dx%d @ %.3f m/px, origin (%.2f, %.2f)'
          % (info.width, info.height, info.resolution,
             info.origin.position.x, info.origin.position.y))
    if info.width == 0 or info.height == 0:
        r.set(BLOCKED, '/map received but empty (0 cells)')
    return r


def check_nav2_action(args, ctx):
    r = CheckResult('C9 Nav2 action server', PHASE_NAV)
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose

    # wait_for_server() only probes the action server's advertised services
    # and topics; it never sends a goal, so the robot cannot move.
    client = ActionClient(ctx.node, NavigateToPose, args.nav2_action_name)
    try:
        available = client.wait_for_server(timeout_sec=args.nav2_timeout)
    finally:
        client.destroy()

    if available:
        r.add('%s action server reachable (wait_for_server only, no goal sent)'
              % args.nav2_action_name)
    else:
        r.set(BLOCKED, '%s action server not reachable within %.1fs -- '
                       'nav_real.launch.py (or bt_navigator inside it) is not '
                       'running' % (args.nav2_action_name, args.nav2_timeout))
    return r


# --------------------------------------------------------------------------
# driver
# --------------------------------------------------------------------------

ENV_CHECKS = [check_host_env, check_fastdds_profile]
ROS_CHECKS = {
    PHASE_BASE: [check_base_topics, check_odom_tf, check_battery, check_camera],
    PHASE_NAV: [check_map_tf, check_map_topic, check_nav2_action],
}


def phases_for(phase):
    """env -> [env]; base -> [env, base]; nav -> [env, base, nav]."""
    return PHASE_ORDER[:PHASE_ORDER.index(phase) + 1]


def build_arg_parser():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--phase', choices=PHASE_ORDER + ['all'], default=PHASE_BASE,
                   help='How far to check. "env" = host env + FastDDS profile '
                        'only (no network, no ROS). "base" (default) adds '
                        'everything the robot-side bringup must provide '
                        'before Nav2 is launched. "nav"/"all" additionally '
                        'require map TF, /map and navigate_to_pose, which '
                        'only exist once Nav2 is up.')
    p.add_argument('--robot-ip', default=None,
                   help='Robot IP for the reachability check. Falls back to '
                        '$TB3_ROBOT_IP, then to the first non-local FastDDS '
                        'initial peer.')
    p.add_argument('--skip-network', action='store_true',
                   help='Deprecated alias for --phase env.')
    p.add_argument('--json', action='store_true', help='Emit JSON instead of text.')
    p.add_argument('--exit-zero-on-warn', action='store_true',
                   help='Exit 0 even when some check WARNs (still 2 on BLOCKED). '
                        'For wrapper scripts that only care about blockers.')
    p.add_argument('--expect-domain', type=int, default=EXPECTED_DOMAIN_ID,
                   help='ROS_DOMAIN_ID the robot bringup uses (-1 to skip '
                        'the check). Default %(default)s.')
    p.add_argument('--ping-timeout', type=float, default=1.5)
    p.add_argument('--ping-count', type=int, default=3,
                   help='ICMP packets to average the RTT over.')
    p.add_argument('--ssh-timeout', type=float, default=2.0)
    p.add_argument('--rtt-warn-ms', type=float, default=200.0,
                   help='Warn above this ping RTT (default %(default)s ms).')
    p.add_argument('--discovery-wait', type=float, default=6.0,
                   help='Max seconds to wait for topics to appear in the '
                        'graph. Polled, so a fast discovery returns early. '
                        'Unicast discovery over campus WiFi needs more than '
                        'the LAN-typical 2s.')
    p.add_argument('--rate-window', type=float, default=3.0,
                   help='Seconds to sample a topic to estimate its rate.')
    p.add_argument('--battery-timeout', type=float, default=6.0,
                   help='Seconds to wait for one /battery_state message '
                        '(OpenCR publishes it slowly).')
    p.add_argument('--tf-timeout', type=float, default=6.0)
    p.add_argument('--map-timeout', type=float, default=6.0)
    p.add_argument('--nav2-timeout', type=float, default=6.0)
    p.add_argument('--nav2-action-name', default='navigate_to_pose')
    p.add_argument('--min-scan-valid-fraction', type=float, default=0.10,
                   help='Warn if fewer than this fraction of /scan returns '
                        'are finite and in range.')
    p.add_argument('--warn-voltage', type=float, default=WARN_VOLTAGE)
    p.add_argument('--block-voltage', type=float, default=BLOCK_VOLTAGE)
    return p


# Names of the ROS checks, so a phase that is not run can be reported as
# SKIPPED without importing rclpy or touching the network.
CHECK_NAMES = {
    PHASE_BASE: ['C3 base topics + rates', 'C4 odom TF', 'C5 battery',
                 'C6 camera'],
    PHASE_NAV: ['C7 map TF (localization)', 'C8 /map',
                'C9 Nav2 action server'],
}


def resolve_phase(args):
    """Normalise --phase / the deprecated --skip-network into one phase name."""
    if args.skip_network:
        return PHASE_ENV
    if args.phase == 'all':
        return PHASE_NAV
    return args.phase


def run_checks(args):
    args.phase = resolve_phase(args)
    active = phases_for(args.phase)

    results = [fn(args, None) for fn in ENV_CHECKS]

    args.resolved_robot_ip, args.resolved_robot_ip_source = \
        resolve_robot_ip(args, results)

    if PHASE_BASE in active:
        results.append(check_robot_reachability(args, None))
    else:
        results.append(CheckResult('C2 robot reachability / SSH port',
                                   PHASE_BASE).skip('skipped (--phase env)'))

    ros_phases = [ph for ph in active if ph in ROS_CHECKS]
    if ros_phases:
        ctx = RosCtx()
        ctx.start()
        try:
            for ph in ros_phases:
                for fn in ROS_CHECKS[ph]:
                    results.append(fn(args, ctx))
        finally:
            ctx.stop()

    for ph in PHASE_ORDER:
        if ph in ROS_CHECKS and ph not in ros_phases:
            for name in CHECK_NAMES[ph]:
                results.append(CheckResult(name, ph).skip(
                    'skipped (--phase %s)' % args.phase))

    return results, active


def overall_status(results):
    overall = READY
    for res in results:
        if _SEVERITY[res.status] > _SEVERITY[overall]:
            overall = res.status
    return overall


def exit_code_for(overall, exit_zero_on_warn=False):
    if overall == BLOCKED:
        return EXIT_BLOCKED
    if overall == WARN:
        return EXIT_OK if exit_zero_on_warn else EXIT_WARN
    return EXIT_OK


def render_text(results, overall, phase, code):
    lines = ['=== TB3 Preflight Healthcheck (phase=%s) ===' % phase]
    for res in results:
        lines.append('[%s] %s' % (res.name, res.status))
        for line in res.lines:
            lines.append('    - %s' % line)
    label = {READY: 'READY', WARN: 'READY WITH WARNINGS',
             BLOCKED: 'NOT READY'}[overall]
    lines.append('Overall: %s (%s, exit %d)' % (label, overall, code))
    if overall == BLOCKED:
        blockers = [r.name for r in results if r.status == BLOCKED]
        lines.append('Blocked check(s): %s' % ', '.join(blockers))
        if phase == PHASE_BASE:
            lines.append('Fix the above before launching nav_real.launch.py; '
                         'if the robot side is down, run on the robot: '
                         'bash ~/tb3_robot_start.sh')
        elif phase == PHASE_NAV:
            lines.append('The base layer may still be fine -- rerun with '
                         '--phase base to confirm, then launch '
                         'nav_real.launch.py on the laptop.')
    return '\n'.join(lines)


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    results, active = run_checks(args)
    overall = overall_status(results)
    code = exit_code_for(overall, args.exit_zero_on_warn)

    if args.json:
        print(json.dumps({
            'overall': overall,
            'phase': args.phase,
            'phases_run': active,
            'exit_code': code,
            'robot_ip': getattr(args, 'resolved_robot_ip', None),
            'checks': [res.to_dict() for res in results],
        }, indent=2))
    else:
        print(render_text(results, overall, args.phase, code))
    return code


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('interrupted', file=sys.stderr)
        sys.exit(EXIT_INTERRUPTED)
    except Exception as exc:  # noqa: BLE001 - tool errors must not look like WARN
        print('tb3_healthcheck: tool error: %s: %s'
              % (type(exc).__name__, exc), file=sys.stderr)
        sys.exit(EXIT_TOOL_ERROR)
