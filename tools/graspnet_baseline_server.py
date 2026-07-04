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
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
from pathlib import Path
import sys

import numpy as np


def make_server(host, port, backend):
    server = ThreadingHTTPServer((host, int(port)), GraspNetBaselineHTTPHandler)
    server.backend = backend
    return server


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
        try:
            length = int(self.headers.get('Content-Length', '0'))
            payload = json.loads(self.rfile.read(length).decode('utf-8'))
            candidates = self.server.backend.predict(payload)
            decoded = decode_rgbd_payload(payload)
            self._send_json(
                200,
                {
                    'ok': True,
                    'backend': self.server.backend.name,
                    'frame_id': decoded['frame_id'],
                    'stamp_sec': decoded['stamp_sec'],
                    'candidates': candidates,
                },
            )
        except Exception as exc:
            self._send_json(200, {'ok': False, 'backend': self.server.backend.name, 'error': str(exc)})

    def log_message(self, fmt, *args):
        sys.stderr.write('[%s] %s\n' % (self.log_date_time_string(), fmt % args))

    def _send_json(self, status, payload):
        data = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class MockGraspNetBackend:
    name = 'mock'

    def health(self):
        return {'ok': True, 'backend': self.name}

    def predict(self, payload):
        decoded = decode_rgbd_payload(payload)
        depth = decoded['depth_raw'].astype(np.float32)
        intr = decoded['intrinsics']
        valid = depth > 0
        if not np.any(valid):
            return []
        ys, xs = np.nonzero(valid)
        idx = len(xs) // 2
        u = float(xs[idx])
        v = float(ys[idx])
        z = float(depth[int(v), int(u)]) * float(intr['depth_scale'])
        x = (u - float(intr['cx'])) * z / float(intr['fx'])
        y = (v - float(intr['cy'])) * z / float(intr['fy'])
        return [
            {
                'score': 1.0,
                'width_m': 0.05,
                'translation_m': [x, y, z],
                'rotation_matrix': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            }
        ]


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
    ):
        self.baseline_root = Path(baseline_root).expanduser()
        self.checkpoint = Path(checkpoint).expanduser()
        self.device_name = str(device or 'cuda:0')
        self.num_view = int(num_view)
        self.num_points = int(num_points)
        self.collision_thresh = float(collision_thresh)
        self.collision_voxel_size = float(collision_voxel_size)
        self.loaded = False

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
        ckpt = torch.load(str(self.checkpoint), map_location=self.device)
        state_dict = ckpt['model_state_dict'] if isinstance(ckpt, dict) and 'model_state_dict' in ckpt else ckpt
        self.net.load_state_dict(state_dict)
        self.net.eval()
        self.loaded = True
        return self

    def predict(self, payload):
        self.load()
        decoded = decode_rgbd_payload(payload)
        model_input, scene_points = self._build_model_input(decoded)
        batch_data = self.collate_fn([model_input])
        batch_data = _to_device(batch_data, self.device)
        with self.torch.no_grad():
            end_points = self.net(batch_data)
            grasp_preds = self.pred_decode(end_points)
        preds = grasp_preds[0].detach().cpu().numpy()
        grasp_group = self.GraspGroup(preds)
        if len(grasp_group) == 0:
            return []
        grasp_group.nms()
        if self.collision_thresh >= 0.0:
            detector = self.ModelFreeCollisionDetector(scene_points, voxel_size=self.collision_voxel_size)
            collision_mask = detector.detect(grasp_group, collision_thresh=self.collision_thresh)
            grasp_group = grasp_group[~collision_mask]
        if len(grasp_group) == 0:
            return []
        grasp_group.sort_by_score()
        max_candidates = max(1, int(decoded['max_candidates']))
        count = min(max_candidates, len(grasp_group))
        return [_grasp_to_response(grasp_group[i]) for i in range(count)]

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
        indices = _sample_indices(len(points), self.num_points)
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
        for path in (self.baseline_root, self.baseline_root / 'utils'):
            text = str(path)
            if text not in sys.path:
                sys.path.insert(0, text)


def decode_rgbd_payload(payload):
    if payload.get('encoding') != 'npz_base64':
        raise ValueError('unsupported payload encoding: %s' % payload.get('encoding'))
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
    }


def _sample_indices(point_count, num_points):
    rng = np.random.default_rng()
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
    raise RuntimeError('failed to import GraspGroup from graspnetAPI')


def _grasp_to_response(grasp):
    return {
        'score': float(grasp.score),
        'width_m': float(getattr(grasp, 'width', 0.0) or 0.0),
        'translation_m': np.asarray(grasp.translation, dtype=float).reshape(3).tolist(),
        'rotation_matrix': np.asarray(grasp.rotation_matrix, dtype=float).reshape(3, 3).tolist(),
    }


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
        )
    if args.warmup and hasattr(backend, 'load'):
        backend.load()
    server = make_server(args.host, args.port, backend)
    print('Alicia GraspNet baseline server listening on http://%s:%d (%s)' % (args.host, args.port, backend.name), flush=True)
    server.serve_forever()


if __name__ == '__main__':
    main()
