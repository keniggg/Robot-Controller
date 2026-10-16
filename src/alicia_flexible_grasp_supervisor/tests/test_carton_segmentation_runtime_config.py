from pathlib import Path

import yaml


CONFIG_PATH = Path(__file__).resolve().parents[1] / 'config' / 'camera.yaml'


def _perception_config():
    with CONFIG_PATH.open('r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)['perception']


def test_default_yolo_resolution_preserves_small_carton_recall():
    assert _perception_config()['yolo_imgsz'] == 640


def test_default_tracking_profile_rejects_background_false_positives():
    config = _perception_config()
    assert config['yolo_conf'] == 0.50
    assert config['tracking_max_jump_px'] == 80.0
    assert config['tracking_switch_confirmations'] == 5
