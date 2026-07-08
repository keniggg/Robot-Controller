import json
import urllib.error
import urllib.parse
import urllib.request


def validate_mujoco_digital_twin_url(server_url):
    normalized = str(server_url or '').strip().rstrip('/')
    if not normalized:
        raise ValueError('mujoco digital twin server_url is empty')
    if '<' in normalized or '>' in normalized:
        raise ValueError(
            'mujoco digital twin server_url contains placeholder angle brackets; '
            'replace it with a real URL such as http://192.168.26.1:9000'
        )
    parsed = urllib.parse.urlparse(normalized)
    if parsed.scheme not in ('http', 'https') or not parsed.netloc:
        raise ValueError('mujoco digital twin server_url must start with http:// or https:// and include host:port')
    return normalized


class MujocoDigitalTwinClient:
    def __init__(self, server_url, timeout_sec=5.0):
        self.server_url = validate_mujoco_digital_twin_url(server_url)
        self.timeout_sec = float(timeout_sec)

    def health(self):
        return self._request_json('/health', None)

    def sync_joint_state(self, joint_state):
        payload = joint_state_payload(joint_state)
        return self._request_json('/sync_joint_state', payload)

    def simulate_grasp(self, payload):
        return self._request_json('/simulate_grasp', payload)

    def _request_json(self, path, payload):
        url = self.server_url + path
        data = None
        method = 'GET'
        headers = {'Accept': 'application/json'}
        if payload is not None:
            data = json.dumps(payload).encode('utf-8')
            method = 'POST'
            headers['Content-Type'] = 'application/json'
        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError('mujoco digital twin HTTP %s: %s' % (exc.code, body)) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError('mujoco digital twin connection failed: %s' % exc) from exc


def build_simulation_payload(
    joint_state=None,
    object_pose=None,
    grasp_plan=None,
    gripper_width_m=0.05,
    object_model=None,
):
    payload = {
        'gripper_width_m': float(gripper_width_m),
        'object_model': dict(object_model or {}),
    }
    if joint_state is not None:
        payload.update(joint_state_payload(joint_state))
    if object_pose is not None:
        payload['object_pose_base'] = object_pose_payload(object_pose, object_model=object_model)
    if grasp_plan is not None:
        payload['grasp_sequence_base'] = grasp_plan_payload(grasp_plan)
    return payload


def joint_state_payload(joint_state):
    return {
        'joint_names': [str(name) for name in getattr(joint_state, 'name', [])],
        'joint_positions': [float(value) for value in getattr(joint_state, 'position', [])],
    }


def object_pose_payload(object_pose, object_model=None):
    pose_stamped = getattr(object_pose, 'pose_base', object_pose)
    payload = pose_payload(pose_stamped)
    model = dict(object_model or {})
    size = model.get('size_xyz_m') or getattr(object_pose, 'size_xyz_m', None)
    payload.update(
        {
            'label': str(getattr(object_pose, 'label', model.get('label', 'object')) or 'object'),
            'confidence': float(getattr(object_pose, 'confidence', 1.0) or 0.0),
            'size_xyz_m': [float(v) for v in (size or [0.10, 0.06, 0.035])],
        }
    )
    return payload


def grasp_plan_payload(grasp_plan):
    names = ['pregrasp', 'approach', 'grasp', 'lift']
    poses = list(getattr(grasp_plan, 'poses', []) or [])
    frame_id = getattr(getattr(grasp_plan, 'header', None), 'frame_id', 'base_link') or 'base_link'
    result = []
    for index, pose in enumerate(poses[:4]):
        stamped = _pose_stamped_from_pose(pose, frame_id)
        item = pose_payload(stamped)
        item['name'] = names[index] if index < len(names) else 'pose_%d' % index
        result.append(item)
    return result


def pose_payload(pose_stamped):
    pose = getattr(pose_stamped, 'pose', pose_stamped)
    position = getattr(pose, 'position')
    orientation = getattr(pose, 'orientation')
    return {
        'frame_id': str(getattr(getattr(pose_stamped, 'header', None), 'frame_id', 'base_link') or 'base_link'),
        'position': [float(position.x), float(position.y), float(position.z)],
        'quaternion_xyzw': [
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        ],
    }


def _pose_stamped_from_pose(pose, frame_id):
    class Header:
        pass

    class PoseStampedLike:
        pass

    stamped = PoseStampedLike()
    stamped.header = Header()
    stamped.header.frame_id = frame_id
    stamped.pose = pose
    return stamped
