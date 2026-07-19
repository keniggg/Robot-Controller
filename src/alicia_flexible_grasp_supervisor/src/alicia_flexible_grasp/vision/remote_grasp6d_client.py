import base64
from copy import deepcopy
from dataclasses import dataclass
import io
import json
import time
from types import MappingProxyType
import urllib.error
import urllib.parse
import urllib.request

import numpy as np
from tf.transformations import quaternion_from_matrix


GRASPNET_DEPTH_BINS_M = (0.01, 0.02, 0.03, 0.04)
GRASPNET_DEPTH_TOLERANCE_M = 1.0e-6
GRASPNET_ROTATION_TOLERANCE = 1.0e-5
GRASP6D_PROTOCOL_VERSION = 3
GRASP6D_CANDIDATE_FIELDS = (
    'score',
    'width_m',
    'height_m',
    'depth_m',
    'translation_m',
    'rotation_matrix',
)
GRASP6D_PERFORMANCE_FIELDS = (
    'server_receive_sec',
    'server_send_sec',
    'preprocess_ms',
    'inference_ms',
    'postprocess_ms',
    'server_total_ms',
    'gpu_allocated_mb',
    'gpu_reserved_mb',
    'gpu_peak_allocated_mb',
)
GRASP6D_SNAPSHOT_STAMP_TOLERANCE_SEC = 1.0e-9


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


@dataclass(frozen=True)
class RemotePredictionBundle:
    """One correlated prediction result with request-local diagnostics."""

    request_id: int
    snapshot_stamp_sec: float
    candidates: tuple
    diagnostics: object
    performance: object
    encode_ms: float
    transport_ms: float
    decode_ms: float

    def __post_init__(self):
        request_id, stamp = _validate_request_correlation(
            self.request_id,
            self.snapshot_stamp_sec,
        )
        object.__setattr__(self, 'request_id', request_id)
        object.__setattr__(self, 'snapshot_stamp_sec', stamp)
        object.__setattr__(
            self,
            'candidates',
            tuple(deepcopy(candidate) for candidate in (self.candidates or ())),
        )
        object.__setattr__(
            self,
            'diagnostics',
            _immutable_json_mapping(self.diagnostics),
        )
        object.__setattr__(
            self,
            'performance',
            _immutable_json_mapping(self.performance),
        )
        for name in ('encode_ms', 'transport_ms', 'decode_ms'):
            object.__setattr__(
                self,
                name,
                _finite_nonnegative_number(getattr(self, name), name),
            )


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
    request_id,
    snapshot_stamp_sec,
    frame_id='camera_link',
    stamp_sec=0.0,
    max_candidates=20,
    max_gripper_width_m=0.0,
    candidate_width_tolerance_m=0.0,
):
    request_id, snapshot_stamp_sec = _validate_request_correlation(
        request_id,
        snapshot_stamp_sec,
    )
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
        'request_id': request_id,
        'snapshot_stamp_sec': snapshot_stamp_sec,
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
    request_id, snapshot_stamp_sec = _validate_request_correlation(
        payload.get('request_id'),
        payload.get('snapshot_stamp_sec'),
    )
    return {
        'color_bgr': color_bgr,
        'depth_raw': depth_raw,
        'request_id': request_id,
        'snapshot_stamp_sec': snapshot_stamp_sec,
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


def validate_predict_protocol_envelope(
    response,
    expected_request_id,
    expected_snapshot_stamp_sec,
):
    """Validate the exact unified WSL ``/predict`` response contract."""
    if not isinstance(response, dict):
        raise ValueError('remote /predict response must be a JSON object')
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
    ok = response.get('ok')
    if type(ok) is not bool:
        raise ValueError(
            'remote /predict ok must be a JSON boolean, got %r' % ok
        )
    candidates = response.get('candidates')
    if not isinstance(candidates, list):
        raise ValueError('remote /predict candidates must be a JSON list')
    if any(not isinstance(candidate, dict) for candidate in candidates):
        raise ValueError(
            'remote /predict candidates must contain only JSON objects'
        )
    diagnostics = response.get('diagnostics')
    if not isinstance(diagnostics, dict):
        raise ValueError('remote /predict diagnostics must be a JSON object')
    expected_request_id, expected_snapshot_stamp_sec = _validate_request_correlation(
        expected_request_id,
        expected_snapshot_stamp_sec,
    )
    response_request_id, response_snapshot_stamp_sec = _validate_request_correlation(
        response.get('request_id'),
        response.get('snapshot_stamp_sec'),
    )
    if (
        response_request_id != expected_request_id
        or abs(response_snapshot_stamp_sec - expected_snapshot_stamp_sec)
        > GRASP6D_SNAPSHOT_STAMP_TOLERANCE_SEC
    ):
        raise ValueError(
            'remote /predict correlation mismatch: expected request_id=%d '
            'snapshot_stamp_sec=%.17g, got request_id=%d '
            'snapshot_stamp_sec=%.17g'
            % (
                expected_request_id,
                expected_snapshot_stamp_sec,
                response_request_id,
                response_snapshot_stamp_sec,
            )
        )
    for field in GRASP6D_PERFORMANCE_FIELDS:
        _finite_nonnegative_number(response.get(field), field)
    try:
        # Validate the complete object, including detached/unknown diagnostics.
        # A direct test double or future transport must not hide NaN/Infinity
        # outside the fields validated explicitly above.
        json.dumps(response, allow_nan=False)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            'remote /predict response must be fully strict-JSON serializable'
        ) from exc
    return response


class RemoteGrasp6DClient:
    def __init__(self, server_url, timeout_sec=3.0, require_candidate_depth=True):
        self.server_url = validate_remote_grasp6d_url(server_url)
        self.timeout_sec = float(timeout_sec)
        self.require_candidate_depth = bool(require_candidate_depth)
        self.last_diagnostics = {}
        self.last_performance = {}
        self._timing_clock = time.perf_counter

    def health(self):
        return self._request_json('/health', None)

    def predict(
        self,
        color_bgr,
        depth_raw,
        intrinsics,
        request_id,
        snapshot_stamp_sec,
        frame_id='camera_link',
        stamp_sec=0.0,
        max_candidates=20,
        max_gripper_width_m=0.0,
        candidate_width_tolerance_m=0.0,
    ):
        self.last_diagnostics = {}
        self.last_performance = {}
        bundle = self.predict_bundle(
            color_bgr,
            depth_raw,
            intrinsics,
            request_id=request_id,
            snapshot_stamp_sec=snapshot_stamp_sec,
            frame_id=frame_id,
            stamp_sec=stamp_sec,
            max_candidates=max_candidates,
            max_gripper_width_m=max_gripper_width_m,
            candidate_width_tolerance_m=candidate_width_tolerance_m,
        )
        self.last_diagnostics = dict(bundle.diagnostics)
        self.last_performance = dict(bundle.performance)
        return list(bundle.candidates)

    def predict_bundle(
        self,
        color_bgr,
        depth_raw,
        intrinsics,
        request_id,
        snapshot_stamp_sec,
        frame_id='camera_link',
        stamp_sec=0.0,
        max_candidates=20,
        max_gripper_width_m=0.0,
        candidate_width_tolerance_m=0.0,
    ):
        """Return an immutable result without mutating legacy ``last_*``."""

        encode_started = self._timing_clock()
        payload = encode_rgbd_payload(
            color_bgr,
            depth_raw,
            intrinsics,
            request_id=request_id,
            snapshot_stamp_sec=snapshot_stamp_sec,
            frame_id=frame_id,
            stamp_sec=stamp_sec,
            max_candidates=max_candidates,
            max_gripper_width_m=max_gripper_width_m,
            candidate_width_tolerance_m=candidate_width_tolerance_m,
        )
        request_data = self._encode_json_payload(payload)
        encode_ms = max(
            0.0,
            (self._timing_clock() - encode_started) * 1000.0,
        )

        transport_started = self._timing_clock()
        if self._request_json_is_overridden():
            response = self._request_json('/predict', payload)
            response_bytes = None
        else:
            response = None
            response_bytes = self._request_bytes('/predict', request_data)
        transport_ms = max(
            0.0,
            (self._timing_clock() - transport_started) * 1000.0,
        )

        decode_started = self._timing_clock()
        if response is None:
            response = self._decode_json_response(response_bytes)
        validate_predict_protocol_envelope(
            response,
            expected_request_id=request_id,
            expected_snapshot_stamp_sec=snapshot_stamp_sec,
        )
        diagnostics = _immutable_json_mapping(response['diagnostics'])
        performance = MappingProxyType(
            {
                field: float(response[field])
                for field in GRASP6D_PERFORMANCE_FIELDS
            }
        )
        candidates = tuple(
            decode_remote_grasp_response(
                response,
                require_candidate_depth=self.require_candidate_depth,
            )
        )
        decode_ms = max(
            0.0,
            (self._timing_clock() - decode_started) * 1000.0,
        )
        validated_request_id, validated_stamp = _validate_request_correlation(
            request_id,
            snapshot_stamp_sec,
        )
        return RemotePredictionBundle(
            request_id=validated_request_id,
            snapshot_stamp_sec=validated_stamp,
            candidates=candidates,
            diagnostics=diagnostics,
            performance=performance,
            encode_ms=encode_ms,
            transport_ms=transport_ms,
            decode_ms=decode_ms,
        )

    def _request_json_is_overridden(self):
        return (
            '_request_json' in self.__dict__
            or type(self)._request_json is not RemoteGrasp6DClient._request_json
        )

    def _request_json(self, path, payload):
        data = None if payload is None else self._encode_json_payload(payload)
        return self._decode_json_response(self._request_bytes(path, data))

    @staticmethod
    def _encode_json_payload(payload):
        return json.dumps(payload, allow_nan=False).encode('utf-8')

    @staticmethod
    def _decode_json_response(raw):
        return json.loads(
            raw.decode('utf-8'),
            parse_constant=_reject_nonstandard_json_constant,
        )

    def _request_bytes(self, path, data):
        url = self.server_url + path
        method = 'GET'
        headers = {'Accept': 'application/json'}
        if data is not None:
            method = 'POST'
            headers['Content-Type'] = 'application/json'
        request = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(request, timeout=self.timeout_sec) as response:
                return response.read()
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError('remote grasp6d HTTP %s: %s' % (exc.code, body)) from exc
        except urllib.error.URLError as exc:
            raise RuntimeError('remote grasp6d connection failed: %s' % exc) from exc


def _freeze_json_value(value):
    if isinstance(value, dict):
        return MappingProxyType(
            {str(key): _freeze_json_value(item) for key, item in value.items()}
        )
    if isinstance(value, list):
        return tuple(_freeze_json_value(item) for item in value)
    return value


def _immutable_json_mapping(value):
    return _freeze_json_value(dict(value or {}))


def _reject_nonstandard_json_constant(value):
    raise ValueError(
        'remote /predict response contains non-standard JSON constant %s'
        % value
    )


def _validate_request_correlation(request_id, snapshot_stamp_sec):
    if type(request_id) is not int or request_id <= 0:
        raise ValueError('request_id must be a positive integer')
    if isinstance(snapshot_stamp_sec, (bool, np.bool_)):
        raise ValueError('snapshot_stamp_sec must be a finite positive number')
    try:
        snapshot_stamp_sec = float(snapshot_stamp_sec)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            'snapshot_stamp_sec must be a finite positive number'
        ) from exc
    if not np.isfinite(snapshot_stamp_sec) or snapshot_stamp_sec <= 0.0:
        raise ValueError('snapshot_stamp_sec must be a finite positive number')
    return request_id, snapshot_stamp_sec


def _finite_nonnegative_number(value, field_name):
    if type(value) not in (int, float):
        raise ValueError(
            'remote /predict %s must be a finite non-negative JSON number'
            % field_name
        )
    result = float(value)
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(
            'remote /predict %s must be a finite non-negative JSON number'
            % field_name
        )
    return result


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
