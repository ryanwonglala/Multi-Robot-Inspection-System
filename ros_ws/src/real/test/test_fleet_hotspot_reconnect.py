"""Pure tests for the fleet hotspot profile generator."""

import importlib.util
import os
import sys
import xml.etree.ElementTree as ET


_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'scripts', 'fleet_hotspot_reconnect.py')


def _load():
    spec = importlib.util.spec_from_file_location(
        'fleet_hotspot_reconnect', _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules['fleet_hotspot_reconnect'] = module
    spec.loader.exec_module(module)
    return module


fleet = _load()


def _addresses(xml_text):
    root = ET.fromstring(xml_text)
    return [
        element.text for element in root.iter()
        if element.tag.rsplit('}', 1)[-1] == 'address'
    ]


def test_laptop_profile_contains_both_robots_and_loopback():
    text = fleet.fastdds_profile(['10.71.66.208', '10.71.66.198'])
    assert _addresses(text) == [
        '10.71.66.208', '10.71.66.198', '127.0.0.1']


def test_robot_profile_contains_laptop_and_loopback():
    text = fleet.fastdds_profile(['10.71.66.1'])
    assert _addresses(text) == ['10.71.66.1', '127.0.0.1']
    assert 'is_default_profile="true"' in text
    assert '<maxInitialPeersRange>32</maxInitialPeersRange>' in text
    assert '<useBuiltinTransports>false</useBuiltinTransports>' in text
