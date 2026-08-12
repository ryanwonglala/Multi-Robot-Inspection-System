#!/usr/bin/env python3
"""Refresh Laptop, TB3, and Jetson Fast DDS peers on a phone hotspot."""

import argparse
import concurrent.futures
import datetime
import ipaddress
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile


TB3 = {
    'name': 'TB3',
    'mac': 'd4:54:8b:2e:c2:2d',
    'user': 'tb',
    'profile': '/home/tb/fastdds_robot.xml',
}
JETSON = {
    'name': 'Jetson',
    'mac': '48:8f:4c:d4:4d:82',
    'user': 'nvidia',
    'profile': '/home/nvidia/fastdds_fleet.xml',
}


def run(command, **kwargs):
    return subprocess.run(command, text=True, **kwargs)


def default_network():
    routes = json.loads(run(
        ['ip', '-j', '-4', 'route', 'show', 'default'],
        check=True, capture_output=True).stdout)
    if not routes:
        raise RuntimeError('no IPv4 default route; connect Laptop to hotspot first')
    route = routes[0]
    interface = route['dev']
    addresses = json.loads(run(
        ['ip', '-j', '-4', 'address', 'show', 'dev', interface],
        check=True, capture_output=True).stdout)
    for info in addresses:
        for addr in info.get('addr_info', []):
            if addr.get('family') == 'inet' and addr.get('scope') == 'global':
                local_ip = addr['local']
                network = ipaddress.ip_network(
                    f"{local_ip}/{addr['prefixlen']}", strict=False)
                return interface, local_ip, network
    raise RuntimeError(f'no global IPv4 address on {interface}')


def ping(ip):
    return run(
        ['ping', '-n', '-c', '1', '-W', '1', str(ip)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL).returncode == 0


def neighbors_by_mac(interface):
    entries = json.loads(run(
        ['ip', '-j', 'neigh', 'show', 'dev', interface],
        check=True, capture_output=True).stdout)
    return {
        entry.get('lladdr', '').lower(): entry['dst']
        for entry in entries if entry.get('lladdr')
    }


def discover(interface, network):
    hosts = list(network.hosts())
    if len(hosts) > 1022:
        raise RuntimeError(
            f'{network} is too large to scan safely; pass both device IPs')
    with concurrent.futures.ThreadPoolExecutor(max_workers=48) as pool:
        list(pool.map(ping, hosts))
    neighbors = neighbors_by_mac(interface)
    return {
        device['name']: neighbors.get(device['mac'])
        for device in (TB3, JETSON)
    }


def ssh_base(device, ip):
    return [
        'ssh',
        '-o', 'BatchMode=yes',
        '-o', 'ConnectTimeout=5',
        '-o', 'StrictHostKeyChecking=accept-new',
        f"{device['user']}@{ip}",
    ]


def verify_device(device, ip):
    command = (
        "printf 'hostname=%s\\n' \"$(hostname)\"; "
        f"grep -ihx '{device['mac']}' /sys/class/net/*/address"
    )
    result = run(ssh_base(device, ip) + [command], capture_output=True)
    if result.returncode != 0 or device['mac'] not in result.stdout.lower():
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(
            f"{device['name']} at {ip} failed SSH/MAC verification: {detail}")
    return result.stdout.splitlines()[0]


def fastdds_profile(peer_ips):
    locators = '\n'.join(
        f'''            <locator>
              <udpv4>
                <address>{ip}</address>
              </udpv4>
            </locator>''' for ip in [*peer_ips, '127.0.0.1'])
    return f'''<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>udp_wide_peers</transport_id>
        <type>UDPv4</type>
        <maxInitialPeersRange>32</maxInitialPeersRange>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="unicast_peer" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>udp_wide_peers</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
        <builtin>
          <initialPeersList>
{locators}
          </initialPeersList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>
'''


def update_local(path, content, dry_run):
    old = path.read_text(encoding='utf-8') if path.exists() else None
    if old == content:
        return 'unchanged'
    if dry_run:
        return 'would update'
    path.parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    backup = None
    if path.exists():
        backup = path.with_name(f'{path.name}.bak-{stamp}')
        shutil.copy2(path, backup)
    with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=path.parent,
            prefix=f'.{path.name}.', delete=False) as handle:
        handle.write(content)
        temporary = handle.name
    os.chmod(temporary, 0o600)
    os.replace(temporary, path)
    return f'updated (backup: {backup})' if backup else 'created'


REMOTE_WRITER = r'''
import datetime
import os
from pathlib import Path
import shutil
import sys
import tempfile

path = Path(sys.argv[1])
content = sys.stdin.read()
old = path.read_text(encoding='utf-8') if path.exists() else None
if old == content:
    print('unchanged')
    raise SystemExit(0)
path.parent.mkdir(parents=True, exist_ok=True)
backup = None
if path.exists():
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    backup = path.with_name(f'{path.name}.bak-{stamp}')
    shutil.copy2(path, backup)
with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', dir=path.parent,
        prefix=f'.{path.name}.', delete=False) as handle:
    handle.write(content)
    temporary = handle.name
os.chmod(temporary, 0o600)
os.replace(temporary, path)
print(f'updated (backup: {backup})' if backup else 'created')
'''


def update_remote(device, ip, content, dry_run):
    current = run(
        ssh_base(device, ip) + [f"cat {device['profile']}"],
        capture_output=True)
    old = current.stdout if current.returncode == 0 else None
    if old == content:
        return 'unchanged'
    if dry_run:
        return 'would update'
    result = run(
        ssh_base(device, ip) +
        [f"python3 -c {shell_quote(REMOTE_WRITER)} {device['profile']}"],
        input=content,
        capture_output=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"could not update {device['name']} profile: "
            f"{result.stderr.strip() or result.stdout.strip()}")
    return result.stdout.strip()


def shell_quote(text):
    return "'" + text.replace("'", "'\"'\"'") + "'"


def stop_local_daemon():
    run([
        'bash', '-lc',
        'source /opt/ros/humble/setup.bash && ros2 daemon stop'
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser(
        description='Refresh Fast DDS peers for Laptop + TB3 + Jetson.')
    parser.add_argument('--tb3-ip', help='skip TB3 discovery')
    parser.add_argument('--jetson-ip', help='skip Jetson discovery')
    parser.add_argument(
        '--restart-tb3', action='store_true',
        help='restart TB3 bringup and camera after updating profiles')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='discover and verify all devices without changing files')
    parser.add_argument(
        '--laptop-profile', type=Path,
        default=Path.home() / 'fastdds_fleet.xml')
    parser.add_argument(
        '--state-file', type=Path,
        default=Path.home() / '.ros' / 'fleet_hotspot.env')
    args = parser.parse_args()

    try:
        interface, laptop_ip, network = default_network()
        print(f'Laptop: {laptop_ip} on {interface} ({network})')
        explicit = {'TB3': args.tb3_ip, 'Jetson': args.jetson_ip}
        found = {}
        if not all(explicit.values()):
            found = discover(interface, network)

        ips = {}
        for device in (TB3, JETSON):
            ip = explicit[device['name']] or found.get(device['name'])
            if not ip:
                raise RuntimeError(
                    f"{device['name']} MAC {device['mac']} was not found; "
                    'confirm all devices appear in the hotspot client list')
            if ipaddress.ip_address(ip) not in network:
                raise RuntimeError(f"{device['name']} address {ip} is not on {network}")
            identity = verify_device(device, ip)
            ips[device['name']] = ip
            print(f"{device['name']}: {ip} ({device['mac']}, {identity})")

        laptop_content = fastdds_profile(
            [ips['TB3'], ips['Jetson']])
        robot_content = fastdds_profile([laptop_ip])
        local_result = update_local(
            args.laptop_profile.expanduser(), laptop_content, args.dry_run)
        tb3_result = update_remote(
            TB3, ips['TB3'], robot_content, args.dry_run)
        jetson_result = update_remote(
            JETSON, ips['Jetson'], robot_content, args.dry_run)
        state_content = (
            f"export TB3_ROBOT_IP={ips['TB3']}\n"
            f"export JETSON_IP={ips['Jetson']}\n")
        state_result = update_local(
            args.state_file.expanduser(), state_content, args.dry_run)
        print(f'Laptop Fast DDS: {local_result}')
        print(f'TB3 Fast DDS:    {tb3_result}')
        print(f'Jetson Fast DDS: {jetson_result}')
        print(f'Fleet IP state:  {state_result}')

        if not args.dry_run:
            stop_local_daemon()
        if args.restart_tb3 and not args.dry_run:
            result = run(
                ssh_base(TB3, ips['TB3']) +
                ['bash /home/tb/tb3_robot_start.sh'])
            if result.returncode != 0:
                raise RuntimeError('TB3 bringup restart failed')
            print('TB3 bringup/camera restarted')

        print('source ~/roboinspec_ws/ros_ws/src/real/scripts/env_fleet.sh')
        return 0
    except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
