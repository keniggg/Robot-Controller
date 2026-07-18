#!/usr/bin/env python3
"""Standalone GraspNet baseline HTTP server for WSL2 GPU inference.

Run this in the WSL2 conda environment, not inside the ROS VM:

  conda activate grasp6d118
  python tools/graspnet_baseline_server.py \
    --baseline-root /home/lv/grasp6d_ws/graspnet-baseline \
    --checkpoint /home/lv/grasp6d_ws/checkpoints/checkpoint-rs.tar \
    --host 0.0.0.0 --port 8000 --device cuda:0
"""
import argparse
import base64
import collections.abc
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import sys
import threading
import time
import types

import numpy as np


GRASP6D_PROTOCOL_VERSION = 3
CANDIDATE_FIELDS = ['score', 'width_m', 'height_m', 'depth_m', 'translation_m', 'rotation_matrix']
PERFORMANCE_FIELDS = (
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
MEBIBYTE = float(1024 * 1024)


@dataclass(frozen=True)
class PredictionBatch:
    request_id: int
    snapshot_stamp_sec: float
    candidates: tuple
    diagnostics: dict
    performance: dict


def install_torch_six_compat():
    """Provide the private torch._six module expected by old GraspNet baseline code."""
    if 'torch._six' in sys.modules:
        return sys.modules['torch._six']
    module = types.ModuleType('torch._six')
    module.container_abcs = collections.abc
    module.string_classes = (str, bytes)
    module.int_classes = int
    module.inf = math.inf
    module.nan = math.nan
    module.FileNotFoundError = FileNotFoundError
    module.PY3 = True
    sys.modules['torch._six'] = module
    return module


def make_server(host, port, backend):
    server = ThreadingHTTPServer((host, int(port)), GraspNetBaselineHTTPHandler)
    server.backend = backend
    return server


def _predict_success_response(batch, backend_name):
    request_id, snapshot_stamp_sec = _validate_request_correlation(
        {
            'request_id': getattr(batch, 'request_id', None),
            'snapshot_stamp_sec': getattr(batch, 'snapshot_stamp_sec', None),
        }
    )
    performance = getattr(batch, 'performance', None)
    if not isinstance(performance, dict):
        raise ValueError('prediction batch performance must be a dictionary')
    normalized_performance = {
        field: _finite_nonnegative_performance(performance.get(field), field)
        for field in PERFORMANCE_FIELDS
    }
    response = {
        'ok': True,
        'backend': str(backend_name),
        'protocol_version': GRASP6D_PROTOCOL_VERSION,
        'candidate_fields': list(CANDIDATE_FIELDS),
        'request_id': request_id,
        'snapshot_stamp_sec': snapshot_stamp_sec,
        'candidates': list(batch.candidates),
        'diagnostics': dict(batch.diagnostics),
        **normalized_performance,
    }
    json.dumps(response, allow_nan=False)
    return response


def _predict_failure_response(backend_name, error, payload=None):
    request_id, snapshot_stamp_sec = _recover_request_correlation(payload)
    return {
        'ok': False,
        'backend': str(backend_name),
        'protocol_version': GRASP6D_PROTOCOL_VERSION,
        'candidate_fields': list(CANDIDATE_FIELDS),
        'request_id': request_id,
        'snapshot_stamp_sec': snapshot_stamp_sec,
        'candidates': [],
        'diagnostics': {},
        **{field: 0.0 for field in PERFORMANCE_FIELDS},
        'error': str(error),
    }


class GraspNetBaselineHTTPHandler(BaseHTTPRequestHandler):
    server_version = 'AliciaGraspNetBaselineHTTP/1.0'

    def do_GET(self):
        if self.path != '/health':
            self._send_json(404, {'ok': False, 'error': 'unknown path'})
            return
        self._send_json(200, self.server.backend.health())

    def do_POST(self):
        if self.path != '/predict':
            self._send_json(404, {'ok': False, 'error': 'unknown path'})
            return
        payload = None
        try:
            length = int(self.headers.get('Content-Length', '0'))
            payload = json.loads(self.rfile.read(length).decode('utf-8'))
            batch = self.server.backend.predict_batch(payload)
            self._send_json(200, _predict_success_response(batch, self.server.backend.name))
        except Exception as exc:
            self._send_json(
                200,
                _predict_failure_response(self.server.backend.name, exc, payload),
            )

    def log_message(self, fmt, *args):
        sys.stderr.write('[%s] %s\n' % (self.log_date_time_string(), fmt % args))

    def _send_json(self, status, payload):
        data = json.dumps(payload, allow_nan=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class MockGraspNetBackend:
    name = 'mock'

    def health(self):
        return {
            'ok': True,
            'backend': self.name,
            'protocol_version': GRASP6D_PROTOCOL_VERSION,
            'candidate_fields': CANDIDATE_FIELDS,
        }

    def predict_batch(self, payload):
        server_receive_sec = time.time()
        server_started = time.perf_counter()
        preprocess_started = time.perf_counter()
        decoded = decode_rgbd_payload(payload)
        depth = decoded['depth_raw'].astype(np.float32)
        intr = decoded['intrinsics']
        valid = depth > 0
        preprocess_ms = _elapsed_ms(preprocess_started)
        inference_ms = 0.0
        postprocess_started = time.perf_counter()
        if not np.any(valid):
            candidates = ()
        else:
            ys, xs = np.nonzero(valid)
            idx = len(xs) // 2
            u = float(xs[idx])
            v = float(ys[idx])
            z = float(depth[int(v), int(u)]) * float(intr['depth_scale'])
            x = (u - float(intr['cx'])) * z / float(intr['fx'])
            y = (v - float(intr['cy'])) * z / float(intr['fy'])
            candidates = (
                {
                    'score': 1.0,
                    'width_m': 0.05,
                    'height_m': 0.02,
                    'depth_m': 0.02,
                    'translation_m': [x, y, z],
                    'rotation_matrix': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                },
            )
        diagnostics = {'returned': len(candidates)}
        postprocess_ms = _elapsed_ms(postprocess_started)
        performance = _performance_snapshot(
            server_receive_sec=server_receive_sec,
            server_started=server_started,
            preprocess_ms=preprocess_ms,
            inference_ms=inference_ms,
            postprocess_ms=postprocess_ms,
        )
        return PredictionBatch(
            request_id=decoded['request_id'],
            snapshot_stamp_sec=decoded['snapshot_stamp_sec'],
            candidates=candidates,
            diagnostics=diagnostics,
            performance=performance,
        )

    def predict(self, payload):
        return list(self.predict_batch(payload).candidates)


class GraspNetBaselineBackend:
    name = 'graspnet_baseline'

    def __init__(
        self,
        baseline_root,
        checkpoint,
        device='cuda:0',
        num_view=300,
        num_points=20000,
        collision_thresh=0.01,
        collision_voxel_size=0.01,
        sampling_seed=0,
        nms_translation_thresh_m=0.03,
        nms_rotation_thresh_deg=30.0,
    ):
        self.baseline_root = Path(baseline_root).expanduser()
        self.checkpoint = Path(checkpoint).expanduser()
        self.device_name = str(device or 'cuda:0')
        self.num_view = int(num_view)
        self.num_points = int(num_points)
        self.collision_thresh = float(collision_thresh)
        self.collision_voxel_size = float(collision_voxel_size)
        self.sampling_seed = int(sampling_seed)
        self.nms_translation_thresh_m = max(0.0, float(nms_translation_thresh_m))
        self.nms_rotation_thresh_deg = max(0.0, float(nms_rotation_thresh_deg))
        self.loaded = False
        self._predict_lock = threading.Lock()
        self.last_diagnostics = {}

    def health(self):
        missing = []
        if not self.baseline_root.is_dir():
            missing.append('baseline_root not found: %s' % self.baseline_root)
        if not self.checkpoint.exists():
            missing.append('checkpoint not found: %s' % self.checkpoint)
        try:
            import torch
            torch_version = str(torch.__version__)
            cuda_available = bool(torch.cuda.is_available())
            torch_cuda = str(getattr(torch.version, 'cuda', None))
        except Exception as exc:
            missing.append('torch import failed: %s' % exc)
            torch_version = 'missing'
            cuda_available = False
            torch_cuda = 'unknown'
        return {
            'ok': not missing,
            'backend': self.name,
            'protocol_version': GRASP6D_PROTOCOL_VERSION,
            'candidate_fields': CANDIDATE_FIELDS,
            'loaded': self.loaded,
            'baseline_root': str(self.baseline_root),
            'checkpoint': str(self.checkpoint),
            'torch': torch_version,
            'torch_cuda_available': cuda_available,
            'torch_cuda_version': torch_cuda,
            'missing': missing,
        }

    def load(self):
        if self.loaded:
            return self
        health = self.health()
        if not health['ok']:
            raise RuntimeError('; '.join(health['missing']))
        self._install_paths()
        try:
            import torch
            install_torch_six_compat()
            from dataset.graspnet_dataset import collate_fn
            from models.graspnet import GraspNet, pred_decode
            from utils.collision_detector import ModelFreeCollisionDetector
            from utils.data_utils import CameraInfo, create_point_cloud_from_depth_image
        except Exception as exc:
            raise RuntimeError('failed to import graspnet-baseline runtime: %s' % exc) from exc

        self.torch = torch
        self.collate_fn = collate_fn
        self.pred_decode = pred_decode
        self.ModelFreeCollisionDetector = ModelFreeCollisionDetector
        self.CameraInfo = CameraInfo
        self.create_point_cloud_from_depth_image = create_point_cloud_from_depth_image
        self.GraspGroup = _import_grasp_group()
        use_cuda = self.device_name.startswith('cuda') and torch.cuda.is_available()
        self.device = torch.device(self.device_name if use_cuda else 'cpu')
        self.net = GraspNet(input_feature_dim=0, num_view=self.num_view, is_training=False)
        self.net.to(self.device)
        try:
            ckpt = torch.load(str(self.checkpoint), map_location=self.device, weights_only=True)
        except TypeError:
            ckpt = torch.load(str(self.checkpoint), map_location=self.device)
        state_dict = ckpt['model_state_dict'] if isinstance(ckpt, dict) and 'model_state_dict' in ckpt else ckpt
        self.net.load_state_dict(state_dict)
        self.net.eval()
        self.loaded = True
        return self

    def predict_batch(self, payload):
        if not self._predict_lock.acquire(False):
            raise RuntimeError('graspnet baseline inference busy; retry after current request')
        server_receive_sec = time.time()
        server_started = time.perf_counter()
        try:
            self.load()
            self.last_diagnostics = {}
            preprocess_started = time.perf_counter()
            decoded = decode_rgbd_payload(payload)
            model_input, scene_points = self._build_model_input(decoded)
            batch_data = self.collate_fn([model_input])
            batch_data = _to_device(batch_data, self.device)
            preprocess_ms = _elapsed_ms(preprocess_started)

            _synchronize_cuda(self.torch, self.device)
            inference_started = time.perf_counter()
            with self.torch.inference_mode():
                end_points = self.net(batch_data)
                grasp_preds = self.pred_decode(end_points)
            _synchronize_cuda(self.torch, self.device)
            inference_ms = _elapsed_ms(inference_started)

            postprocess_started = time.perf_counter()
            preds = grasp_preds[0].detach().cpu().numpy()
            grasp_group = self.GraspGroup(preds)
            diagnostics = {'raw_candidates': int(len(grasp_group))}
            if len(grasp_group) > 0:
                grasp_group.nms(
                    translation_thresh=self.nms_translation_thresh_m,
                    rotation_thresh=math.radians(self.nms_rotation_thresh_deg),
                )
                diagnostics['after_nms'] = int(len(grasp_group))
                diagnostics['nms_translation_thresh_m'] = self.nms_translation_thresh_m
                diagnostics['nms_rotation_thresh_deg'] = self.nms_rotation_thresh_deg
                if len(grasp_group) > 0:
                    if self.collision_thresh >= 0.0:
                        detector = self.ModelFreeCollisionDetector(
                            scene_points,
                            voxel_size=self.collision_voxel_size,
                        )
                        collision_mask = detector.detect(
                            grasp_group,
                            collision_thresh=self.collision_thresh,
                        )
                        grasp_group = grasp_group[~collision_mask]
                    diagnostics['after_collision'] = int(len(grasp_group))
                    max_width = float(decoded.get('max_gripper_width_m') or 0.0)
                    width_tolerance = max(
                        0.0,
                        float(decoded.get('candidate_width_tolerance_m') or 0.0),
                    )
                    if max_width > 0.0 and len(grasp_group) > 0:
                        widths = _grasp_group_widths(grasp_group)
                        keep = widths <= (max_width + width_tolerance)
                        diagnostics['width_limit_m'] = max_width
                        diagnostics['width_tolerance_m'] = width_tolerance
                        diagnostics['width_rejected'] = int(np.count_nonzero(~keep))
                        grasp_group = grasp_group[keep]
                        diagnostics['after_width'] = int(len(grasp_group))
                grasp_group.sort_by_score()
            max_candidates = max(1, int(decoded['max_candidates']))
            count = min(max_candidates, len(grasp_group))
            diagnostics['returned'] = int(count)
            self.last_diagnostics = diagnostics
            candidates = tuple(
                _grasp_to_response(grasp_group[index])
                for index in range(count)
            )
            postprocess_ms = _elapsed_ms(postprocess_started)
            performance = _performance_snapshot(
                server_receive_sec=server_receive_sec,
                server_started=server_started,
                preprocess_ms=preprocess_ms,
                inference_ms=inference_ms,
                postprocess_ms=postprocess_ms,
                **_cuda_memory_snapshot(self.torch, self.device)
            )
            return PredictionBatch(
                request_id=decoded['request_id'],
                snapshot_stamp_sec=decoded['snapshot_stamp_sec'],
                candidates=candidates,
                diagnostics=dict(diagnostics),
                performance=performance,
            )
        except Exception as exc:
            if _is_cuda_out_of_memory(getattr(self, 'torch', None), exc):
                sys.stderr.write(
                    'CUDA OOM rejected GraspNet request; clearing allocator cache\n'
                )
                _empty_cuda_cache(getattr(self, 'torch', None))
            raise
        finally:
            self._predict_lock.release()

    def predict(self, payload):
        return list(self.predict_batch(payload).candidates)

    def _build_model_input(self, decoded):
        intr = decoded['intrinsics']
        depth = decoded['depth_raw'].astype(np.float32)
        color_rgb = decoded['color_bgr'].astype(np.float32)[:, :, ::-1] / 255.0
        camera = self.CameraInfo(
            int(intr['width']),
            int(intr['height']),
            float(intr['fx']),
            float(intr['fy']),
            float(intr['cx']),
            float(intr['cy']),
            1.0 / float(intr['depth_scale']),
        )
        cloud = self.create_point_cloud_from_depth_image(depth, camera, organized=True)
        valid_mask = depth > 0.0
        points = cloud[valid_mask].astype(np.float32)
        colors = color_rgb[valid_mask].astype(np.float32)
        if points.size == 0:
            raise RuntimeError('no valid depth points')
        indices = _sample_indices(len(points), self.num_points, seed=self.sampling_seed)
        sampled_points = points[indices].astype(np.float32)
        sampled_colors = colors[indices].astype(np.float32)
        return (
            {
                'point_clouds': sampled_points,
                'cloud_colors': sampled_colors,
            },
            points,
        )

    def _install_paths(self):
        for path in (
            self.baseline_root,
            self.baseline_root / 'models',
            self.baseline_root / 'utils',
            self.baseline_root / 'pointnet2',
            self.baseline_root / 'knn',
        ):
            text = str(path)
            if text not in sys.path:
                sys.path.insert(0, text)


def decode_rgbd_payload(payload):
    request_id, snapshot_stamp_sec = _validate_request_correlation(payload)
    if payload.get('encoding') != 'npz_base64':
        raise ValueError('unsupported payload encoding: %s' % payload.get('encoding'))
    raw = base64.b64decode(payload['data_npz_b64'].encode('ascii'))
    with np.load(io.BytesIO(raw), allow_pickle=False) as archive:
        color_bgr = archive['color_bgr']
        depth_raw = archive['depth_raw']
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


def _validate_request_correlation(payload):
    if not isinstance(payload, dict):
        raise ValueError('request payload must be a JSON object')
    request_id = payload.get('request_id')
    if type(request_id) is not int or request_id <= 0:
        raise ValueError('request_id must be a positive integer')
    snapshot_stamp_sec = payload.get('snapshot_stamp_sec')
    if isinstance(snapshot_stamp_sec, (bool, np.bool_)):
        raise ValueError('snapshot_stamp_sec must be a finite positive number')
    try:
        snapshot_stamp_sec = float(snapshot_stamp_sec)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            'snapshot_stamp_sec must be a finite positive number'
        ) from exc
    if not math.isfinite(snapshot_stamp_sec) or snapshot_stamp_sec <= 0.0:
        raise ValueError('snapshot_stamp_sec must be a finite positive number')
    return request_id, snapshot_stamp_sec


def _recover_request_correlation(payload):
    try:
        return _validate_request_correlation(payload)
    except Exception:
        return None, None


def _finite_nonnegative_performance(value, field):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError('%s must be a finite non-negative number' % field)
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(
            '%s must be a finite non-negative number' % field
        ) from exc
    if not math.isfinite(result) or result < 0.0:
        raise ValueError('%s must be a finite non-negative number' % field)
    return result


def _elapsed_ms(started):
    return max(0.0, (time.perf_counter() - started) * 1000.0)


def _performance_snapshot(
    server_receive_sec,
    server_started,
    preprocess_ms,
    inference_ms,
    postprocess_ms,
    gpu_allocated_mb=0.0,
    gpu_reserved_mb=0.0,
    gpu_peak_allocated_mb=0.0,
):
    return {
        'server_receive_sec': max(0.0, float(server_receive_sec)),
        'server_send_sec': max(0.0, float(time.time())),
        'preprocess_ms': max(0.0, float(preprocess_ms)),
        'inference_ms': max(0.0, float(inference_ms)),
        'postprocess_ms': max(0.0, float(postprocess_ms)),
        'server_total_ms': _elapsed_ms(server_started),
        'gpu_allocated_mb': max(0.0, float(gpu_allocated_mb)),
        'gpu_reserved_mb': max(0.0, float(gpu_reserved_mb)),
        'gpu_peak_allocated_mb': max(0.0, float(gpu_peak_allocated_mb)),
    }


def _device_is_cuda(device):
    return str(device or '').lower().startswith('cuda')


def _synchronize_cuda(torch_module, device):
    if not _device_is_cuda(device):
        return
    synchronize = getattr(getattr(torch_module, 'cuda', None), 'synchronize', None)
    if callable(synchronize):
        synchronize(device)


def _cuda_memory_snapshot(torch_module, device):
    values = {
        'gpu_allocated_mb': 0.0,
        'gpu_reserved_mb': 0.0,
        'gpu_peak_allocated_mb': 0.0,
    }
    if not _device_is_cuda(device):
        return values
    cuda = getattr(torch_module, 'cuda', None)
    for field, function_name in (
        ('gpu_allocated_mb', 'memory_allocated'),
        ('gpu_reserved_mb', 'memory_reserved'),
        ('gpu_peak_allocated_mb', 'max_memory_allocated'),
    ):
        function = getattr(cuda, function_name, None)
        if not callable(function):
            continue
        try:
            value = float(function(device)) / MEBIBYTE
        except Exception:
            continue
        if math.isfinite(value) and value >= 0.0:
            values[field] = value
    return values


def _is_cuda_out_of_memory(torch_module, error):
    error_types = []
    for owner in (torch_module, getattr(torch_module, 'cuda', None)):
        error_type = getattr(owner, 'OutOfMemoryError', None)
        if isinstance(error_type, type):
            error_types.append(error_type)
    return bool(error_types) and isinstance(error, tuple(error_types))


def _sample_indices(point_count, num_points, seed=0):
    rng = np.random.default_rng(int(seed))
    if point_count >= num_points:
        return rng.choice(point_count, num_points, replace=False)
    base = np.arange(point_count)
    extra = rng.choice(point_count, num_points - point_count, replace=True)
    return np.concatenate([base, extra], axis=0)


def _to_device(value, device):
    if hasattr(value, 'to'):
        return value.to(device)
    if isinstance(value, dict):
        return {key: _to_device(item, device) for key, item in value.items()}
    if isinstance(value, list):
        return [_to_device(item, device) for item in value]
    if isinstance(value, tuple):
        return tuple(_to_device(item, device) for item in value)
    return value


def _empty_cuda_cache(torch_module):
    try:
        cuda = getattr(torch_module, 'cuda', None)
        empty_cache = getattr(cuda, 'empty_cache', None)
        if callable(empty_cache):
            empty_cache()
    except Exception:
        pass


def _grasp_group_widths(grasp_group):
    try:
        return np.asarray(grasp_group.widths, dtype=np.float32).reshape(-1)
    except Exception:
        return np.asarray([float(getattr(grasp, 'width', 0.0) or 0.0) for grasp in grasp_group], dtype=np.float32)


class FallbackGrasp:
    def __init__(self, grasp_array):
        self.grasp_array = np.asarray(grasp_array, dtype=np.float32).reshape(-1)
        if self.grasp_array.size < 16:
            raise ValueError('grasp prediction must contain at least 16 values')

    @property
    def score(self):
        return float(self.grasp_array[0])

    @property
    def width(self):
        return float(self.grasp_array[1])

    @property
    def height(self):
        return float(self.grasp_array[2])

    @property
    def depth(self):
        return float(self.grasp_array[3])

    @property
    def rotation_matrix(self):
        return self.grasp_array[4:13].reshape(3, 3)

    @property
    def translation(self):
        return self.grasp_array[13:16]


class FallbackGraspGroup:
    """Minimal GraspNetAPI-compatible container for inference-only postprocessing."""

    def __init__(self, grasp_group_array=None):
        if grasp_group_array is None:
            array = np.zeros((0, 17), dtype=np.float32)
        elif isinstance(grasp_group_array, FallbackGraspGroup):
            array = grasp_group_array.grasp_group_array.copy()
        elif isinstance(grasp_group_array, (list, tuple)) and grasp_group_array and isinstance(grasp_group_array[0], FallbackGrasp):
            array = np.vstack([grasp.grasp_array for grasp in grasp_group_array]).astype(np.float32)
        else:
            array = np.asarray(grasp_group_array, dtype=np.float32)
            if array.size == 0:
                array = np.zeros((0, 17), dtype=np.float32)
            if array.ndim == 1:
                array = array.reshape(1, -1)
        if array.size and array.shape[1] < 16:
            raise ValueError('grasp group array must have at least 16 columns')
        self.grasp_group_array = array.astype(np.float32, copy=False)

    def __len__(self):
        return int(self.grasp_group_array.shape[0])

    def __iter__(self):
        for row in self.grasp_group_array:
            yield FallbackGrasp(row)

    def __getitem__(self, index):
        selected = self.grasp_group_array[index]
        if isinstance(index, (int, np.integer)):
            return FallbackGrasp(selected)
        return FallbackGraspGroup(selected)

    @property
    def scores(self):
        return self.grasp_group_array[:, 0]

    @property
    def widths(self):
        return self.grasp_group_array[:, 1]

    @property
    def heights(self):
        return self.grasp_group_array[:, 2]

    @property
    def depths(self):
        return self.grasp_group_array[:, 3]

    @property
    def rotation_matrices(self):
        return self.grasp_group_array[:, 4:13].reshape((-1, 3, 3))

    @property
    def translations(self):
        return self.grasp_group_array[:, 13:16]

    def nms(self, translation_thresh=0.03, rotation_thresh=math.radians(30.0)):
        """Greedy GraspNet-compatible SE(3) NMS for inference without graspnetAPI."""
        if len(self) <= 1:
            return self
        translation_thresh = max(0.0, float(translation_thresh))
        rotation_thresh = max(0.0, float(rotation_thresh))
        order = np.argsort(-self.scores, kind='stable')
        translations = self.translations
        rotations = self.rotation_matrices
        kept = []
        jaw_symmetry = np.diag([1.0, -1.0, -1.0]).astype(np.float32)
        for index in order:
            duplicate = False
            for kept_index in kept:
                distance = float(np.linalg.norm(translations[index] - translations[kept_index]))
                if distance > translation_thresh:
                    continue
                angle = _rotation_distance_rad(rotations[index], rotations[kept_index])
                symmetric_angle = _rotation_distance_rad(
                    rotations[index],
                    rotations[kept_index].dot(jaw_symmetry),
                )
                if min(angle, symmetric_angle) <= rotation_thresh:
                    duplicate = True
                    break
            if not duplicate:
                kept.append(int(index))
        self.grasp_group_array = self.grasp_group_array[np.asarray(kept, dtype=np.int64)]
        return self

    def sort_by_score(self):
        if len(self) > 1:
            order = np.argsort(-self.scores)
            self.grasp_group_array = self.grasp_group_array[order]
        return self


def _import_grasp_group():
    candidates = (
        ('graspnetAPI', 'GraspGroup'),
        ('graspnetAPI.grasp', 'GraspGroup'),
        ('graspnetAPI.graspnet_eval', 'GraspGroup'),
    )
    for module_name, attr in candidates:
        try:
            module = __import__(module_name, fromlist=[attr])
            return getattr(module, attr)
        except Exception:
            pass
    sys.stderr.write('WARNING: graspnetAPI not found; using internal inference-only FallbackGraspGroup\n')
    return FallbackGraspGroup


def _grasp_to_response(grasp):
    return {
        'score': float(grasp.score),
        'width_m': float(getattr(grasp, 'width', 0.0) or 0.0),
        'height_m': float(getattr(grasp, 'height', 0.0) or 0.0),
        'depth_m': float(getattr(grasp, 'depth', 0.0) or 0.0),
        'translation_m': np.asarray(grasp.translation, dtype=float).reshape(3).tolist(),
        'rotation_matrix': np.asarray(grasp.rotation_matrix, dtype=float).reshape(3, 3).tolist(),
    }


def _rotation_distance_rad(first, second):
    relative = np.asarray(first, dtype=np.float64).reshape(3, 3).T.dot(
        np.asarray(second, dtype=np.float64).reshape(3, 3)
    )
    cosine = np.clip((float(np.trace(relative)) - 1.0) * 0.5, -1.0, 1.0)
    return float(math.acos(cosine))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Alicia remote GraspNet baseline inference server')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--baseline-root', default=str(Path.home() / 'grasp6d_ws' / 'graspnet-baseline'))
    parser.add_argument('--checkpoint', default=str(Path.home() / 'grasp6d_ws' / 'checkpoints' / 'checkpoint-rs.tar'))
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--num-view', type=int, default=300)
    parser.add_argument('--num-points', type=int, default=20000)
    parser.add_argument('--collision-thresh', type=float, default=0.01)
    parser.add_argument('--collision-voxel-size', type=float, default=0.01)
    parser.add_argument('--sampling-seed', type=int, default=0)
    parser.add_argument('--nms-translation-thresh-m', type=float, default=0.03)
    parser.add_argument('--nms-rotation-thresh-deg', type=float, default=30.0)
    parser.add_argument('--mock', action='store_true', help='Serve deterministic fake grasps for network testing')
    parser.add_argument('--warmup', action='store_true', help='Load model before accepting requests')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.mock:
        backend = MockGraspNetBackend()
    else:
        backend = GraspNetBaselineBackend(
            baseline_root=args.baseline_root,
            checkpoint=args.checkpoint,
            device=args.device,
            num_view=args.num_view,
            num_points=args.num_points,
            collision_thresh=args.collision_thresh,
            collision_voxel_size=args.collision_voxel_size,
            sampling_seed=args.sampling_seed,
            nms_translation_thresh_m=args.nms_translation_thresh_m,
            nms_rotation_thresh_deg=args.nms_rotation_thresh_deg,
        )
    if args.warmup and hasattr(backend, 'load'):
        backend.load()
    server = make_server(args.host, args.port, backend)
    print('Alicia GraspNet baseline server listening on http://%s:%d (%s)' % (args.host, args.port, backend.name), flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
