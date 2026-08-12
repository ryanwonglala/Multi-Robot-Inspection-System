#!/usr/bin/env python3
"""Find the TB3 on the current hotspot and refresh both Fast DDS peers."""

import argparse
import concurrent.futures
import datetime
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


TB3_MAC = 'd4:54:8b:2e:c2:2d'
TB3_USER = 'tb'
REMOTE_PROFILE = '/home/tb/fastdds_robot.xml'
ADDRESS_RE = re.compile(r'(<address>)(?!127\.0\.0\.1)([^<]+)(</address>)')


def run(command, **kwargs):
    return subprocess.run(command, text=True, **kwargs)


def default_network():
    routes = json.loads(run(
        ['ip', '-j', '-4', 'route', 'show', 'default'],
        check=True, capture_output=True).stdout)
    if not routes:
        raise RuntimeError('no IPv4 default route; connect the laptop to the hotspot first')
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


def neighbor_for_mac(interface):
    neighbors = json.loads(run(
        ['ip', '-j', 'neigh', 'show', 'dev', interface],
        check=True, capture_output=True).stdout)
    for entry in neighbors:
        if entry.get('lladdr', '').lower() == TB3_MAC:
            return entry['dst']
    return None


def discover_robot(interface, network):
    # Android has repeatedly assigned this TB3 host number 208. Try it first,
    # but still verify the MAC and fall back to a full /24 sweep.
    preferred = network.network_address + 208
    if preferred in network:
        ping(preferred)
        found = neighbor_for_mac(interface)
        if found:
            return found

    hosts = list(network.hosts())
    if len(hosts) > 1022:
        raise RuntimeError(
            f'{network} is too large to scan safely; pass --robot-ip explicitly')
    with concurrent.futures.ThreadPoolExecutor(max_workers=48) as pool:
        list(pool.map(ping, hosts))
    return neighbor_for_mac(interface)


def ssh_base(robot_ip):
    return [
        'ssh',
        '-o', 'BatchMode=yes',
        '-o', 'ConnectTimeout=5',
        '-o', 'StrictHostKeyChecking=accept-new',
        f'{TB3_USER}@{robot_ip}',
    ]


def verify_robot(robot_ip):
    command = (
        "printf 'hostname=%s\\n' \"$(hostname)\"; "
        f"grep -ihx '{TB3_MAC}' /sys/class/net/*/address"
    )
    result = run(
        ssh_base(robot_ip) + [command],
        capture_output=True)
    if result.returncode != 0 or TB3_MAC not in result.stdout.lower():
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(
            f'{robot_ip} answered SSH but did not verify as the known TB3: {detail}')
    return result.stdout.splitlines()[0]


def replaced_profile(text, peer_ip):
    updated, count = ADDRESS_RE.subn(
        lambda match: f'{match.group(1)}{peer_ip}{match.group(3)}',
        text)
    if count != 1:
        raise RuntimeError(
            f'expected exactly one non-loopback <address>, found {count}')
    return updated


def update_local_profile(path, peer_ip, dry_run):
    original = path.read_text(encoding='utf-8')
    updated = replaced_profile(original, peer_ip)
    if updated == original:
        return 'unchanged'
    if dry_run:
        return f'would update peer to {peer_ip}'

    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
    backup = path.with_name(f'{path.name}.bak-{stamp}')
    shutil.copy2(path, backup)
    with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', dir=path.parent,
            prefix=f'.{path.name}.', delete=False) as handle:
        handle.write(updated)
        temporary = handle.name
    os.chmod(temporary, path.stat().st_mode)
    os.replace(temporary, path)
    return f'updated (backup: {backup})'


REMOTE_UPDATER = r'''
import datetime
import os
from pathlib import Path
import re
import shutil
import sys
import tempfile

path = Path(sys.argv[1])
peer_ip = sys.argv[2]
pattern = re.compile(r'(<address>)(?!127\.0\.0\.1)([^<]+)(</address>)')
original = path.read_text(encoding='utf-8')
updated, count = pattern.subn(
    lambda match: f'{match.group(1)}{peer_ip}{match.group(3)}', original)
if count != 1:
    raise SystemExit(
        f'expected exactly one non-loopback <address>, found {count}')
if updated == original:
    print('unchanged')
    raise SystemExit(0)
stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
backup = path.with_name(f'{path.name}.bak-{stamp}')
shutil.copy2(path, backup)
with tempfile.NamedTemporaryFile(
        mode='w', encoding='utf-8', dir=path.parent,
        prefix=f'.{path.name}.', delete=False) as handle:
    handle.write(updated)
    temporary = handle.name
os.chmod(temporary, path.stat().st_mode)
os.replace(temporary, path)
print(f'updated (backup: {backup})')
'''


def update_remote_profile(robot_ip, laptop_ip, dry_run):
    if dry_run:
        return f'would update peer to {laptop_ip}'
    result = run(
        ssh_base(robot_ip) +
        [f'python3 - {REMOTE_PROFILE} {laptop_ip}'],
        input=REMOTE_UPDATER,
        capture_output=True)
    if result.returncode != 0:
        raise RuntimeError(
            f'could not update robot profile: {result.stderr.strip()}')
    return result.stdout.strip()


def stop_local_daemon():
    run([
        'bash', '-lc',
        'source /opt/ros/humble/setup.bash && ros2 daemon stop'
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    parser = argparse.ArgumentParser(
        description='Reconnect Fast DDS after the phone hotspot changes subnet.')
    parser.add_argument(
        '--robot-ip',
        help='skip discovery and use this address (the robot MAC is still verified)')
    parser.add_argument(
        '--restart-robot', action='store_true',
        help='restart TB3 bringup/camera after updating the profiles')
    parser.add_argument(
        '--dry-run', action='store_true',
        help='discover and verify, but do not change either profile')
    parser.add_argument(
        '--laptop-profile', type=Path,
        default=Path.home() / 'fastdds_laptop.xml')
    args = parser.parse_args()

    try:
        interface, laptop_ip, network = default_network()
        print(f'laptop: {laptop_ip} on {interface} ({network})')

        robot_ip = args.robot_ip or discover_robot(interface, network)
        if not robot_ip:
            raise RuntimeError(
                f'TB3 MAC {TB3_MAC} was not found on {network}; '
                'confirm it appears in the phone hotspot client list')
        if ipaddress.ip_address(robot_ip) not in network:
            raise RuntimeError(f'robot address {robot_ip} is not on {network}')

        identity = verify_robot(robot_ip)
        print(f'robot:  {robot_ip} ({TB3_MAC}, {identity})')

        local_result = update_local_profile(
            args.laptop_profile.expanduser(), robot_ip, args.dry_run)
        remote_result = update_remote_profile(
            robot_ip, laptop_ip, args.dry_run)
        print(f'laptop Fast DDS: {local_result}')
        print(f'robot Fast DDS:  {remote_result}')

        if not args.dry_run:
            stop_local_daemon()
        if args.restart_robot and not args.dry_run:
            result = run(
                ssh_base(robot_ip) + ['bash /home/tb/tb3_robot_start.sh'])
            if result.returncode != 0:
                raise RuntimeError('robot bringup restart failed')
            print('robot bringup/camera restarted')
        elif not args.dry_run:
            print('robot processes were not restarted; use --restart-robot when needed')

        print(f'export TB3_ROBOT_IP={robot_ip}')
        return 0
    except (OSError, RuntimeError, subprocess.SubprocessError, ValueError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
