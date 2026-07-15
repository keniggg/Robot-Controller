import base64
from dataclasses import dataclass
import io
import json
import urllib.error
import urllib.parse
import urllib.request

import numpy as np
from tf.transformations import quaternion_from_matrix


@dataclass
class RemoteGraspCandidate:
    score: float
    translation_m: np.ndarray
    quaternion_xyzw: np.ndarray
    width_m: float = 0.0
    height_m: float = None
    depth_m: float = None


def validate_remote_grasp6d_url(server_url):
    normalized = str(server_url or '').strip().rstrip('/')
    if not normalized:
        raise ValueError('remote grasp6d server_url is empty')
    if '<' in normalized or '>' in normalized:
        raise ValueError(
            'remote grasp6d server_url contains placeholder angle brackets; '
            'replace it with a real URL such as http://192.168.26.1:8000'
        )
    parsed = urllib.parse.urlparse(normalized)
    if parsed.scheme not in ('http', 'https') or not parsed.netloc:
        raise ValueError('remote grasp6d server_url must start with http:// or https:// and include host:port')
    return normalized


def encode_rgbd_payload(
    color_bgr,
    depth_raw,
    intrinsics,
    frame_id='camera_link',
    stamp_sec=0.0,
    max_candidates=20,
    max_gripper_width_m=0.0,
    candidate_width_tolerance_m=0.0,
):
    buffer = io.BytesIO()
    np.savez_compressed(
        buffer,
        color_bgr=np.asarray(color_bgr, dtype=np.uint8),
        depth_raw=np.asarray(depth_raw),
    )
    intrinsics_dict = {
        'width': int(intrinsics.width),
        'height': int(intrinsics.height),
        'fx': float(intrinsics.fx),
        'fy': float(intrinsics.fy),
        'cx': float(intrinsics.cx),
        'cy': float(intrinsics.cy),
        'depth_scale': float(intrinsics.depth_scale),
    }
    return {
        'encoding': 'npz_base64',
        'data_npz_b64': base64.b64encode(buffer.getvalue()).decode('ascii'),
        'intrinsics': intrinsics_dict,
        'frame_id': str(frame_id or 'camera_link'),
        'stamp_sec': float(stamp_sec or 0.0),
        'max_candidates': int(max_candidates),
        'max_gripper_width_m': float(max_gripper_width_m or 0.0),
        'candidate_width_tolerance_m': float(candidate_width_tolerance_m or 0.0),
    }


def decode_rgbd_payload(payload):
    if payload.get('encoding') != 'npz_base64':
        raise ValueError('unsupported RGB-D payload encoding: %s' % payload.get('encoding'))
    raw = base64.b64decode(payload['data_npz_b64'].encode('ascii'))
    with np.load(io.BytesIO(raw), allow_pickle=False) as archive:
        color_bgr = archive['color_bgr']
        depth_raw = archive['depth_raw']
    return {
        'color_bgr': color_bgr,
        'depth_raw': depth_raw,
        'intrinsics': dict(payload.get('intrinsics') or {}),
        'frame_id': str(payload.get('frame_id') or 'camera_link'),
        'stamp_sec': float(payload.get('stamp_sec') or 0.0),
        'max_candidates': int(payload.get('max_candidates') or 20),
        'max_gripper_width_m': float(payload.get('max_gripper_width_m') or 0.0),
        'candidate_width_tolerance_m': float(payload.get('candidate_width_tolerance_m') or 0.0),
    }


def decode_remote_grasp_response(response):
    if not bool(response.get('ok', False)):
        raise RuntimeError(str(response.get('error') or 'remote 6D grasp server returned failure'))
    candidates = []
    for item in response.get('candidates') or []:
        translation = _vector3(item.get('translation_m'), 'translation_m')
        quat = _candidate_quaternion(item)
        candidates.append(
            RemoteGraspCandidate(
                score=float(item.get('score', 0.0)),
                width_m=float(item.get('width_m', 0.0) or 0.0),
                height_m=(float(item['height_m']) if item.get('height_m') is not None else None),
                depth_m=(float(item['depth_m']) if item.get('depth_m') is not None else None),
                translation_m=translation,
                quaternion_xyzw=quat,
            )
        )
    return candidates


class RemoteGrasp6DClient:
    def __init__(self, server_url, timeout_sec=3.0):
        self.server_url = validate_remote_grasp6d_url(server_url)
        self.timeout_sec = float(timeout_sec)
        self.last_diagnostics = {}

    def health(self):
        return self._request_json('/health', None)

    def predict(
        self,
        color_bgr,
        depth_raw,
        intrinsics,
        frame_id='camera_link',
        stamp_sec=0.0,
        max_candidates=20,
        max_gripper_width_m=0.0,
        candidate_width_tolerance_m=0.0,
    ):
        payload = encode_rgbd_payload(
            color_bgr,
            depth_raw,
            intrinsics,
            frame_id=frame_id,
            stamp_sec=stamp_sec,
            max_candidates=max_candidates,
            max_gripper_width_m=max_gripper_width_m,
            candidate_width_tolerance_m=candidate_width_tolerance_m,
        )
        response = self._request_json('/predict', payload)
        self.last_diagnostics = dict(response.get('diagnostics') or {})
        return decode_remote_grasp_response(response)

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
            raise RuntimeError('remote grasp6d HTTP %s: %s' % (exc.code, body)) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError('remote grasp6d connection failed: %s' % exc) from exc


def _candidate_quaternion(item):
    if item.get('quaternion_xyzw') is not None:
        quat = np.asarray(item.get('quaternion_xyzw'), dtype=float)
        if quat.shape != (4,):
            raise ValueError('quaternion_xyzw must contain 4 values')
        return _normalize_quaternion(quat)
    rotation = np.asarray(item.get('rotation_matrix'), dtype=float)
    if rotation.shape != (3, 3):
        raise ValueError('candidate must contain quaternion_xyzw or 3x3 rotation_matrix')
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = rotation
    return _normalize_quaternion(np.asarray(quaternion_from_matrix(mat), dtype=float))


def _vector3(value, name):
    vec = np.asarray(value, dtype=float)
    if vec.shape != (3,):
        raise ValueError('%s must contain 3 values' % name)
    return vec


def _normalize_quaternion(quat):
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        raise ValueError('quaternion has zero norm')
    quat = quat / norm
    if quat[3] < 0.0:
        quat = -quat
    return quat
