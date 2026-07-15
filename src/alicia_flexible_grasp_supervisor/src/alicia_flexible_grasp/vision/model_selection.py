from copy import deepcopy
from pathlib import Path


DEFAULT_MODEL_PROFILES = {
    'original': {
        'display_name': 'YOLOv8 原模型',
        'model_path': 'yolov8n.pt',
        'target_class_mode': 'description',
    },
    'carton': {
        'display_name': 'Carton 模型',
        'model_path': 'carton_model/best.pt',
        'target_class_mode': 'fixed',
        'target_class': 'carton',
    },
}


def normalize_model_profiles(perception_cfg):
    configured = dict((perception_cfg or {}).get('yolo_models') or {})
    source = configured or DEFAULT_MODEL_PROFILES
    profiles = {}
    for choice, raw_profile in source.items():
        profile = deepcopy(dict(raw_profile or {}))
        mode = str(profile.get('target_class_mode', '')).strip().lower()
        model_path = str(profile.get('model_path', '')).strip()
        if mode not in ('description', 'fixed'):
            raise ValueError('Invalid target_class_mode for %s: %s' % (choice, mode))
        if not model_path:
            raise ValueError('Missing model_path for %s' % choice)
        if mode == 'fixed' and not str(profile.get('target_class', '')).strip():
            raise ValueError('Missing fixed target_class for %s' % choice)
        profile['display_name'] = str(profile.get('display_name', choice))
        profile['model_path'] = model_path
        profile['target_class_mode'] = mode
        profile['target_class'] = str(profile.get('target_class', '')).strip()
        profiles[str(choice)] = profile
    return profiles


def select_yolo_model(perception_cfg, choice, description_target_class):
    profiles = normalize_model_profiles(perception_cfg)
    choice = str(choice or 'original')
    if choice not in profiles:
        raise ValueError('Unknown YOLO model choice: %s' % choice)
    profile = deepcopy(profiles[choice])
    if profile['target_class_mode'] == 'description':
        profile['target_class'] = str(description_target_class or '').strip()
    profile['choice'] = choice
    return profile


def _discover_package_path():
    try:
        import rospkg
        return rospkg.RosPack().get_path('alicia_flexible_grasp_supervisor')
    except Exception:
        return None


def resolve_yolo_model_path(model_path, package_path=None, cwd=None):
    raw_path = str(model_path or '').strip()
    if not raw_path:
        raise ValueError('YOLO model path is empty')
    expanded = Path(raw_path).expanduser()
    if expanded.is_absolute():
        if not expanded.is_file():
            raise FileNotFoundError('YOLO model file not found: %s' % raw_path)
        return str(expanded.resolve())
    if len(expanded.parts) == 1:
        return raw_path

    roots = [Path(cwd).resolve() if cwd is not None else Path.cwd().resolve()]
    package = package_path or _discover_package_path()
    if package:
        package_root = Path(package).resolve()
        roots.append(package_root)
        if package_root.parent.name == 'src':
            roots.append(package_root.parent.parent)

    checked = []
    for root in roots:
        candidate = (root / expanded).resolve()
        if candidate in checked:
            continue
        checked.append(candidate)
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError('YOLO model file not found: %s' % raw_path)
