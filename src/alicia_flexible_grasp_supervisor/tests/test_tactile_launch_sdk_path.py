"""Launch path resolution only; never import/connect the physical SDK."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def test_tactile_config_does_not_supply_an_unexpanded_roslaunch_expression():
    config = yaml.safe_load((PACKAGE / 'config/tactile.yaml').read_text())
    assert config['tactile']['sdk_path'] == ''
    assert config['tactile']['simulate'] is False
    assert config['tactile']['port'] == '/dev/alicia_skin'


def test_sensor_launch_supplies_workspace_relative_sdk_after_loading_yaml():
    launch = ET.parse(str(PACKAGE / 'launch/sensors.launch')).getroot()
    children = list(launch)
    config = next(n for n in children if n.tag == 'rosparam'
                  and n.attrib.get('file', '').endswith('/config/tactile.yaml'))
    param = next(n for n in children if n.tag == 'param'
                 and n.attrib.get('name') == '/tactile/sdk_path')
    assert children.index(param) > children.index(config)
    assert param.attrib['value'] == '$(dirname)/../../Electronic-Skin-ML'
    assert 'catkin_ws' not in param.attrib['value']
