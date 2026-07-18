import base64
from dataclasses import dataclass
import io
import json
import urllib.error
import urllib.parse
import urllib.request

import numpy as np
from tf.transformations import quaternion_from_matrix


GRASPNET_DEPTH_BINS_M = (0.01, 0.02, 0.03, 0.04)
GRASPNET_DEPTH_TOLERANCE_M = 1.0e-6
GRASPNET_ROTATION_TOLERANCE = 1.0e-5
GRASP6D_PROTOCOL_VERSION = 2
GRASP6D_CANDIDATE_FIELDS = (
    'score',
    'width_m',
    'height_m',
    'depth_m',
    'translation_m',
    'rotation_matrix',
)


class CandidateContractError(ValueError):
    """Structured fail-closed error for one remote GraspNet candidate."""

    def __init__(self, code, message):
        self.code = str(code or 'CANDIDATE_CONTRACT_INVALID')
        super().__init__(str(message))


def validate_graspnet_depth_m(value, required=True):
    """Validate the discrete insertion-depth contract emitted by GraspNet."""
    if value is None:
        if bool(required):
            raise CandidateContractError(
                'DEPTH_MISSING',
                'candidate depth_m is required by the GraspNet tool0 contract',
            )
        return None
    if isinstance(value, (bool, np.bool_)):
        raise CandidateContractError(
            'DEPTH_INVALID',
            'candidate depth_m must be a numeric GraspNet depth bin',
        )
    try:
        depth = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CandidateContractError(
            'DEPTH_INVALID',
            'candidate depth_m must be numeric',
        ) from exc
    if not np.isfinite(depth):
        raise CandidateContractError(
            'DEPTH_INVALID',
            'candidate depth_m must be finite',
        )
    nearest = min(GRASPNET_DEPTH_BINS_M, key=lambda item: abs(depth - item))
    if abs(depth - nearest) > GRASPNET_DEPTH_TOLERANCE_M:
        raise CandidateContractError(
            'DEPTH_OUT_OF_RANGE',
            'candidate depth_m %.9g is not a GraspNet depth bin %s'
            % (depth, list(GRASPNET_DEPTH_BINS_M)),
        )
    return float(nearest)


@dataclass
class RemoteGraspCandidate:
    score: float
    # GraspNet's translation is the grasp/contact center in the point cloud.
    # It must not be reinterpreted as the robot flange/TCP position.
    translation_m: np.ndarray
    quaternion_xyzw: np.ndarray
    width_m: float = 0.0
    height_m: float = None
    depth_m: float = None
    # Derived once on the ROS side from ``translation_m + depth * approach``.
    # None is retained for legacy responses that do not carry insertion depth.
    tool0_translation_m: np.ndarray = None


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


def decode_remote_grasp_response(response, require_candidate_depth=True):
    if not bool(response.get('ok', False)):
        raise RuntimeError(str(response.get('error') or 'remote 6D grasp server returned failure'))
    candidates = []
    for item in response.get('candidates') or []:
        translation = _vector3(item.get('translation_m'), 'translation_m')
        quat = _candidate_quaternion(item)
        depth = validate_graspnet_depth_m(
            item.get('depth_m'),
            required=require_candidate_depth,
        )
        candidates.append(
            RemoteGraspCandidate(
                score=float(item.get('score', 0.0)),
                width_m=float(item.get('width_m', 0.0) or 0.0),
                height_m=(float(item['height_m']) if item.get('height_m') is not None else None),
                depth_m=depth,
                translation_m=translation,
                quaternion_xyzw=quat,
            )
        )
    return candidates


def validate_predict_protocol_envelope(response):
    """Validate the exact unified WSL ``/predict`` response contract."""
    if not isinstance(response, dict):
        raise ValueError('remote /predict response must be a JSON object')
    try:
        # Validate the complete object, including detached/unknown diagnostics.
        # A direct test double or future transport must not hide NaN/Infinity
        # outside the candidate fields that are consumed below.
        json.dumps(response, allow_nan=False)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            'remote /predict response must be fully strict-JSON serializable'
        ) from exc
    protocol_version = response.get('protocol_version')
    if (
        type(protocol_version) is not int
        or protocol_version != GRASP6D_PROTOCOL_VERSION
    ):
        raise ValueError(
            'remote /predict protocol_version must be integer %d, got %r'
            % (GRASP6D_PROTOCOL_VERSION, protocol_version)
        )
    candidate_fields = response.get('candidate_fields')
    expected_fields = list(GRASP6D_CANDIDATE_FIELDS)
    if not isinstance(candidate_fields, list) or candidate_fields != expected_fields:
        raise ValueError(
            'remote /predict candidate_fields must exactly equal %r, got %r'
            % (expected_fields, candidate_fields)
        )
    return response


class RemoteGrasp6DClient:
    def __init__(self, server_url, timeout_sec=3.0, require_candidate_depth=True):
        self.server_url = validate_remote_grasp6d_url(server_url)
        self.timeout_sec = float(timeout_sec)
        self.require_candidate_depth = bool(require_candidate_depth)
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
        self.last_diagnostics = {}
        response = self._request_json('/predict', payload)
        validate_predict_protocol_envelope(response)
        self.last_diagnostics = dict(response.get('diagnostics') or {})
        return decode_remote_grasp_response(
            response,
            require_candidate_depth=self.require_candidate_depth,
        )

    def _request_json(self, path, payload):
        url = self.server_url + path
        data = None
        method = 'GET'
        headers = {'Accept': 'application/json'}
        if payload is not None:
            data = json.dumps(payload, allow_nan=False).encode('utf-8')
            method = 'POST'
            headers['Content-Type'] = 'application/json'
        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                return json.loads(
                    response.read().decode('utf-8'),
                    parse_constant=_reject_nonstandard_json_constant,
                )
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError('remote grasp6d HTTP %s: %s' % (exc.code, body)) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError('remote grasp6d connection failed: %s' % exc) from exc


def _reject_nonstandard_json_constant(value):
    raise ValueError(
        'remote /predict response contains non-standard JSON constant %s'
        % value
    )


def _candidate_quaternion(item):
    if item.get('quaternion_xyzw') is not None:
        try:
            quat = np.asarray(item.get('quaternion_xyzw'), dtype=float)
        except (TypeError, ValueError, OverflowError) as exc:
            raise CandidateContractError(
                'ORIENTATION_INVALID',
                'candidate quaternion_xyzw must contain 4 numeric values',
            ) from exc
        if quat.shape != (4,):
            raise CandidateContractError(
                'ORIENTATION_INVALID',
                'candidate quaternion_xyzw must contain 4 values',
            )
        return _normalize_quaternion(quat)
    try:
        rotation = np.asarray(item.get('rotation_matrix'), dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CandidateContractError(
            'ORIENTATION_INVALID',
            'candidate rotation_matrix must contain numeric values',
        ) from exc
    if rotation.shape != (3, 3):
        raise CandidateContractError(
            'ORIENTATION_INVALID',
            'candidate must contain quaternion_xyzw or 3x3 rotation_matrix',
        )
    if not np.all(np.isfinite(rotation)):
        raise CandidateContractError(
            'ORIENTATION_INVALID',
            'candidate rotation_matrix must contain only finite values',
        )
    if not np.allclose(
        rotation.T @ rotation,
        np.eye(3, dtype=float),
        rtol=0.0,
        atol=GRASPNET_ROTATION_TOLERANCE,
    ):
        raise CandidateContractError(
            'ORIENTATION_INVALID',
            'candidate rotation_matrix must be orthonormal',
        )
    determinant = float(np.linalg.det(rotation))
    if (
        not np.isfinite(determinant)
        or abs(determinant - 1.0) > GRASPNET_ROTATION_TOLERANCE
    ):
        raise CandidateContractError(
            'ORIENTATION_INVALID',
            'candidate rotation_matrix must be right-handed with determinant +1',
        )
    mat = np.eye(4, dtype=float)
    mat[:3, :3] = rotation
    try:
        quat = np.asarray(quaternion_from_matrix(mat), dtype=float)
    except Exception as exc:
        raise CandidateContractError(
            'ORIENTATION_INVALID',
            'candidate rotation_matrix could not be converted to a quaternion',
        ) from exc
    return _normalize_quaternion(quat)


def _vector3(value, name):
    vec = np.asarray(value, dtype=float)
    if vec.shape != (3,):
        raise ValueError('%s must contain 3 values' % name)
    return vec


def _normalize_quaternion(quat):
    if not np.all(np.isfinite(quat)):
        raise CandidateContractError(
            'ORIENTATION_INVALID',
            'candidate quaternion_xyzw must contain only finite values',
        )
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(norm) or norm <= 1e-12:
        raise CandidateContractError(
            'ORIENTATION_INVALID',
            'candidate quaternion_xyzw must have a finite non-zero norm',
        )
    quat = quat / norm
    if quat[3] < 0.0:
        quat = -quat
    return quat
