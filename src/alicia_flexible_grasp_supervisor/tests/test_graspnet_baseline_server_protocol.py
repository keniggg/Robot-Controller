#!/usr/bin/env python3
import collections.abc
import importlib.util
import json
import pathlib
import sys
import threading
import tempfile
import types
import unittest
import urllib.request

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[3]
PKG = ROOT / 'src' / 'alicia_flexible_grasp_supervisor'
for path in (PKG, PKG / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision.remote_grasp6d_client import encode_rgbd_payload


SCRIPT = ROOT / 'tools' / 'graspnet_baseline_server.py'
spec = importlib.util.spec_from_file_location('graspnet_baseline_server', str(SCRIPT))
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)


class GraspNetBaselineServerProtocolTest(unittest.TestCase):
    def test_installs_torch_six_compat_for_pytorch_2_baseline_imports(self):
        previous = sys.modules.pop('torch._six', None)
        try:
            compat = server_module.install_torch_six_compat()
            imported = sys.modules['torch._six']

            self.assertIs(compat, imported)
            self.assertIs(imported.container_abcs, collections.abc)
            self.assertEqual(imported.string_classes, (str, bytes))
            self.assertIs(imported.int_classes, int)
            self.assertEqual(imported.inf, float('inf'))
            self.assertTrue(imported.nan != imported.nan)
        finally:
            if previous is None:
                sys.modules.pop('torch._six', None)
            else:
                sys.modules['torch._six'] = previous

    def test_mock_server_serves_health_and_predict(self):
        server = server_module.make_server('127.0.0.1', 0, server_module.MockGraspNetBackend())
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base_url = 'http://127.0.0.1:%d' % server.server_port
            health = self._json_get(base_url + '/health')
            self.assertTrue(health['ok'])
            self.assertEqual(health['backend'], 'mock')

            payload = encode_rgbd_payload(
                np.zeros((4, 4, 3), dtype=np.uint8),
                np.full((4, 4), 1000, dtype=np.uint16),
                CameraIntrinsics(width=4, height=4, fx=100.0, fy=100.0, cx=2.0, cy=2.0, depth_scale=0.001),
                max_candidates=3,
            )
            response = self._json_post(base_url + '/predict', payload)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

        self.assertTrue(response['ok'])
        self.assertEqual(len(response['candidates']), 1)
        self.assertIn('translation_m', response['candidates'][0])
        self.assertIn('rotation_matrix', response['candidates'][0])

    def test_baseline_backend_installs_legacy_baseline_import_paths(self):
        original_path = list(sys.path)
        try:
            with tempfile.TemporaryDirectory() as tmp:
                root = pathlib.Path(tmp) / 'graspnet-baseline'
                for relative in ('models', 'utils', 'pointnet2', 'knn'):
                    (root / relative).mkdir(parents=True)

                backend = server_module.GraspNetBaselineBackend(root, pathlib.Path(tmp) / 'checkpoint-rs.tar')
                backend._install_paths()

                installed = set(sys.path)
                self.assertIn(str(root), installed)
                self.assertIn(str(root / 'models'), installed)
                self.assertIn(str(root / 'utils'), installed)
                self.assertIn(str(root / 'pointnet2'), installed)
                self.assertIn(str(root / 'knn'), installed)
        finally:
            sys.path[:] = original_path

    def test_baseline_backend_load_uses_official_collate_fn_and_constructor(self):
        created = {}

        class FakeCuda:
            @staticmethod
            def is_available():
                return False

        class FakeTorch(types.SimpleNamespace):
            def __init__(self):
                super().__init__(
                    __version__='2.4.1+cu118',
                    cuda=FakeCuda(),
                    version=types.SimpleNamespace(cuda='11.8'),
                    device=lambda name: name,
                    load=lambda _path, map_location=None: {'model_state_dict': {'weight': 1}},
                )

        class FakeGraspNet:
            def __init__(self, **kwargs):
                created['kwargs'] = kwargs

            def to(self, device):
                created['device'] = device

            def load_state_dict(self, state_dict):
                created['state_dict'] = state_dict

            def eval(self):
                created['eval'] = True

        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp) / 'graspnet-baseline'
            root.mkdir()
            checkpoint = pathlib.Path(tmp) / 'checkpoint-rs.tar'
            checkpoint.write_bytes(b'fake checkpoint')
            modules = {
                'torch': FakeTorch(),
                'dataset': types.ModuleType('dataset'),
                'dataset.graspnet_dataset': types.SimpleNamespace(collate_fn=lambda batch: {'batch': batch}),
                'models': types.ModuleType('models'),
                'models.graspnet': types.SimpleNamespace(GraspNet=FakeGraspNet, pred_decode=lambda end_points: []),
                'utils': types.ModuleType('utils'),
                'utils.collision_detector': types.SimpleNamespace(ModelFreeCollisionDetector=object),
                'utils.data_utils': types.SimpleNamespace(CameraInfo=object, create_point_cloud_from_depth_image=lambda depth, camera, organized=True: depth),
                'graspnetAPI': types.SimpleNamespace(GraspGroup=object),
            }

            self._with_modules(modules, lambda: server_module.GraspNetBaselineBackend(root, checkpoint, device='cuda:0').load())

        self.assertEqual(created['kwargs']['input_feature_dim'], 0)
        self.assertFalse('seed_feat_dim' in created['kwargs'])
        self.assertFalse(hasattr(server_module.GraspNetBaselineBackend, 'minkowski_collate_fn'))
        self.assertEqual(created['state_dict'], {'weight': 1})
        self.assertTrue(created['eval'])

    def test_baseline_model_input_contains_only_point_clouds_and_cloud_colors(self):
        backend = server_module.GraspNetBaselineBackend('/tmp/baseline', '/tmp/checkpoint')
        backend.num_points = 4
        backend.CameraInfo = lambda *args: {'camera': args}
        backend.create_point_cloud_from_depth_image = lambda depth, camera, organized=True: np.dstack(
            [
                np.tile(np.arange(depth.shape[1]), (depth.shape[0], 1)),
                np.tile(np.arange(depth.shape[0]).reshape(-1, 1), (1, depth.shape[1])),
                depth.astype(np.float32) * 0.001,
            ]
        )
        decoded = {
            'depth_raw': np.array([[1000, 0], [1200, 1300]], dtype=np.uint16),
            'color_bgr': np.arange(12, dtype=np.uint8).reshape(2, 2, 3),
            'intrinsics': {'width': 2, 'height': 2, 'fx': 100.0, 'fy': 100.0, 'cx': 1.0, 'cy': 1.0, 'depth_scale': 0.001},
            'max_candidates': 1,
        }

        model_input, scene_points = backend._build_model_input(decoded)

        self.assertEqual(set(model_input.keys()), {'point_clouds', 'cloud_colors'})
        self.assertEqual(model_input['point_clouds'].shape, (4, 3))
        self.assertEqual(model_input['cloud_colors'].shape, (4, 3))
        self.assertFalse('coors' in model_input)
        self.assertFalse('feats' in model_input)
        self.assertEqual(scene_points.shape, (3, 3))

    def _json_get(self, url):
        with urllib.request.urlopen(url, timeout=2.0) as response:
            return json.loads(response.read().decode('utf-8'))

    def _json_post(self, url, payload):
        data = json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return json.loads(response.read().decode('utf-8'))

    def _with_modules(self, modules, fn):
        old = {}
        missing = object()
        try:
            for name, module in modules.items():
                old[name] = sys.modules.get(name, missing)
                sys.modules[name] = module
            return fn()
        finally:
            for name, value in old.items():
                if value is missing:
                    sys.modules.pop(name, None)
                else:
                    sys.modules[name] = value


if __name__ == '__main__':
    unittest.main()
