import yaml
from pathlib import Path


def load_yaml(path, default=None):
    p = Path(path).expanduser()
    if not p.exists():
        return default if default is not None else {}
    with p.open('r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}
