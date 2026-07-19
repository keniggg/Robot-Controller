#!/usr/bin/env python3
import builtins
import collections.abc
import contextlib
import importlib.util
import json
import pathlib
import sys
import threading
import tempfile
import types
import unittest
import urllib.request
from unittest import mock

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
    @staticmethod
    def _valid_payload(request_id=1, snapshot_stamp_sec=123.25):
        return encode_rgbd_payload(
            np.zeros((4, 4, 3), dtype=np.uint8),
            np.full((4, 4), 1000, dtype=np.uint16),
            CameraIntrinsics(
                width=4,
                height=4,
                fx=100.0,
                fy=100.0,
                cx=2.0,
                cy=2.0,
                depth_scale=0.001,
            ),
            request_id=request_id,
            snapshot_stamp_sec=snapshot_stamp_sec,
            max_candidates=3,
        )

    @staticmethod
    def _loaded_fake_backend():
        class FakeCuda:
            def __init__(self):
                self.empty_cache = mock.Mock()
                self.synchronize = mock.Mock()
                self.memory_allocated = mock.Mock(return_value=64 * 1024 * 1024)
                self.memory_reserved = mock.Mock(return_value=96 * 1024 * 1024)
                self.max_memory_allocated = mock.Mock(return_value=80 * 1024 * 1024)

        class FakeTorch:
            def __init__(self):
                self.cuda = FakeCuda()

            @staticmethod
            def inference_mode():
                return contextlib.nullcontext()

        class FakePrediction:
            def detach(self):
                return self

            def cpu(self):
                return self

            @staticmethod
            def numpy():
                return np.asarray(
                    [
                        [
                            0.9,
                            0.04,
                            0.02,
                            0.03,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.0,
                            0.0,
                            0.0,
                            1.0,
                            0.1,
                            0.2,
                            0.3,
                            -1.0,
                        ]
                    ],
                    dtype=np.float32,
                )

        backend = server_module.GraspNetBaselineBackend(
            '/tmp/baseline',
            '/tmp/checkpoint',
            device='cuda:0',
            collision_thresh=-1.0,
        )
        fake_torch = FakeTorch()
        backend.load_count = 0

        def load_once():
            if backend.loaded:
                return backend
            backend.load_count += 1
            backend.torch = fake_torch
            backend.device = 'cuda:0'
            backend.collate_fn = lambda rows: {'rows': rows}
            backend.net = mock.Mock(return_value={'end_points': True})
            backend.pred_decode = lambda _end_points: [FakePrediction()]
            backend.GraspGroup = server_module.FallbackGraspGroup
            backend._build_model_input = lambda _decoded: (
                {'point_clouds': np.zeros((4, 3), dtype=np.float32)},
                np.zeros((4, 3), dtype=np.float32),
            )
            backend.loaded = True
            return backend

        backend.load = load_once
        return backend, fake_torch

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
            self.assertEqual(health['protocol_version'], 3)
            self.assertIn('depth_m', health['candidate_fields'])
            self.assertIn('height_m', health['candidate_fields'])

            payload = encode_rgbd_payload(
                np.zeros((4, 4, 3), dtype=np.uint8),
                np.full((4, 4), 1000, dtype=np.uint16),
                CameraIntrinsics(width=4, height=4, fx=100.0, fy=100.0, cx=2.0, cy=2.0, depth_scale=0.001),
                request_id=7,
                snapshot_stamp_sec=123.25,
                max_candidates=3,
            )
            response = self._json_post(base_url + '/predict', payload)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

        self.assertTrue(response['ok'])
        self.assertEqual(response['protocol_version'], 3)
        self.assertEqual(response['request_id'], 7)
        self.assertEqual(response['snapshot_stamp_sec'], 123.25)
        for field in (
            'server_receive_sec',
            'server_send_sec',
            'preprocess_ms',
            'inference_ms',
            'postprocess_ms',
            'server_total_ms',
            'gpu_allocated_mb',
            'gpu_reserved_mb',
            'gpu_peak_allocated_mb',
        ):
            self.assertGreaterEqual(response[field], 0.0)
        self.assertEqual(len(response['candidates']), 1)
        self.assertIn('height_m', response['candidates'][0])
        self.assertIn('depth_m', response['candidates'][0])
        self.assertIn('translation_m', response['candidates'][0])
        self.assertIn('rotation_matrix', response['candidates'][0])

    def test_standalone_server_fails_closed_on_incomplete_batch_performance(self):
        class IncompleteBatchBackend:
            name = 'incomplete_batch'

            @staticmethod
            def predict_batch(payload):
                return server_module.PredictionBatch(
                    request_id=payload['request_id'],
                    snapshot_stamp_sec=payload['snapshot_stamp_sec'],
                    candidates=(),
                    diagnostics={},
                    performance={},
                )

        server = server_module.make_server('127.0.0.1', 0, IncompleteBatchBackend())
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            response = self._json_post(
                'http://127.0.0.1:%d/predict' % server.server_port,
                self._valid_payload(request_id=8),
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

        self.assertFalse(response['ok'])
        self.assertEqual(response['request_id'], 8)
        self.assertEqual(response['snapshot_stamp_sec'], 123.25)
        self.assertIn('server_receive_sec', response['error'])
        for field in server_module.PERFORMANCE_FIELDS:
            self.assertEqual(response[field], 0.0)

    def test_backend_loads_once_and_normal_predict_never_empties_cuda_cache(self):
        backend, fake_torch = self._loaded_fake_backend()

        backend.predict_batch(self._valid_payload(request_id=1))
        backend.predict_batch(self._valid_payload(request_id=2))

        self.assertEqual(backend.load_count, 1)
        self.assertEqual(fake_torch.cuda.empty_cache.call_count, 0)
        self.assertEqual(backend.net.call_count, 2)

    def test_cuda_oom_rejects_request_then_clears_allocator_cache(self):
        backend, fake_torch = self._loaded_fake_backend()
        backend.load()

        class FakeCudaOutOfMemoryError(RuntimeError):
            pass

        fake_torch.OutOfMemoryError = FakeCudaOutOfMemoryError
        backend.net.side_effect = FakeCudaOutOfMemoryError('synthetic cuda oom')

        with self.assertRaisesRegex(FakeCudaOutOfMemoryError, 'synthetic cuda oom'):
            backend.predict_batch(self._valid_payload(request_id=3))

        self.assertEqual(fake_torch.cuda.empty_cache.call_count, 1)

    def test_prediction_batch_has_correlated_timing_and_gpu_memory(self):
        backend, _fake_torch = self._loaded_fake_backend()

        batch = backend.predict_batch(self._valid_payload(request_id=7))

        self.assertIsInstance(batch, server_module.PredictionBatch)
        self.assertEqual(batch.request_id, 7)
        self.assertEqual(batch.snapshot_stamp_sec, 123.25)
        self.assertEqual(len(batch.candidates), 1)
        for field in (
            'server_receive_sec',
            'server_send_sec',
            'preprocess_ms',
            'inference_ms',
            'postprocess_ms',
            'server_total_ms',
            'gpu_allocated_mb',
            'gpu_reserved_mb',
            'gpu_peak_allocated_mb',
        ):
            self.assertTrue(np.isfinite(batch.performance[field]))
            self.assertGreaterEqual(batch.performance[field], 0.0)
        self.assertEqual(batch.performance['gpu_allocated_mb'], 64.0)
        self.assertEqual(batch.performance['gpu_reserved_mb'], 96.0)
        self.assertEqual(batch.performance['gpu_peak_allocated_mb'], 80.0)

    def test_empty_after_nms_skips_collision_detector(self):
        backend, _fake_torch = self._loaded_fake_backend()
        backend.load()

        class EmptyAfterNmsGroup(server_module.FallbackGraspGroup):
            def nms(self, translation_thresh=0.03, rotation_thresh=np.deg2rad(30.0)):
                self.grasp_group_array = np.zeros((0, 17), dtype=np.float32)
                return self

        backend.GraspGroup = EmptyAfterNmsGroup
        backend.collision_thresh = 0.01
        backend.ModelFreeCollisionDetector = mock.Mock(
            side_effect=AssertionError('collision detector must not receive an empty group')
        )

        batch = backend.predict_batch(self._valid_payload(request_id=9))

        self.assertEqual(batch.candidates, ())
        backend.ModelFreeCollisionDetector.assert_not_called()

    def test_backend_uses_nms_returned_group_like_official_graspnet_api(self):
        backend, _fake_torch = self._loaded_fake_backend()
        backend.load()

        class ReturnOnlyNmsGroup(server_module.FallbackGraspGroup):
            def nms(self, translation_thresh=0.03, rotation_thresh=np.deg2rad(30.0)):
                return server_module.FallbackGraspGroup(
                    np.zeros((0, 17), dtype=np.float32)
                )

        backend.GraspGroup = ReturnOnlyNmsGroup

        batch = backend.predict_batch(self._valid_payload(request_id=10))

        self.assertEqual(batch.candidates, ())
        self.assertEqual(batch.diagnostics['raw_candidates'], 1)
        self.assertEqual(batch.diagnostics['after_nms'], 0)

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

    def test_fallback_grasp_group_supports_baseline_postprocessing_without_graspnetapi(self):
        original_import = builtins.__import__

        def blocked_import(name, *args, **kwargs):
            if name.startswith('graspnetAPI'):
                raise ImportError('blocked graspnetAPI for fallback test')
            return original_import(name, *args, **kwargs)

        rows = np.array(
            [
                [0.2, 0.04, 0.02, 0.03, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.10, 0.20, 0.30, -1.0],
                [0.9, 0.05, 0.03, 0.04, 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.40, 0.50, 0.60, -1.0],
            ],
            dtype=np.float32,
        )
        try:
            builtins.__import__ = blocked_import
            GraspGroup = server_module._import_grasp_group()
        finally:
            builtins.__import__ = original_import

        group = GraspGroup(rows)
        self.assertEqual(len(group), 2)
        self.assertEqual(group.translations.shape, (2, 3))
        self.assertEqual(group.rotation_matrices.shape, (2, 3, 3))
        self.assertEqual(group.widths.shape, (2,))
        self.assertEqual(group.heights.shape, (2,))
        self.assertEqual(group.depths.shape, (2,))

        group.nms()
        group.sort_by_score()
        self.assertAlmostEqual(group[0].score, 0.9, places=5)
        filtered = group[np.array([True, False])]
        self.assertEqual(len(filtered), 1)
        response = server_module._grasp_to_response(group[0])
        np.testing.assert_allclose(response['translation_m'], [0.4, 0.5, 0.6], rtol=1e-6, atol=1e-6)
        self.assertAlmostEqual(response['height_m'], 0.03, places=6)
        self.assertAlmostEqual(response['depth_m'], 0.04, places=6)
        self.assertEqual(len(response['rotation_matrix']), 3)

    def test_fallback_nms_removes_pose_duplicates_but_preserves_orientation_diversity(self):
        identity = np.eye(3, dtype=np.float32)
        yaw_90 = np.array(
            [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )

        def row(score, rotation, translation):
            return np.asarray(
                [score, 0.05, 0.02, 0.03]
                + rotation.reshape(-1).tolist()
                + list(translation)
                + [-1.0],
                dtype=np.float32,
            )

        group = server_module.FallbackGraspGroup(
            np.vstack(
                [
                    row(0.90, identity, [0.10, 0.20, 0.30]),
                    row(0.80, identity, [0.105, 0.20, 0.30]),
                    row(0.70, yaw_90, [0.105, 0.20, 0.30]),
                    row(0.60, identity, [0.16, 0.20, 0.30]),
                ]
            )
        )

        group.nms(translation_thresh=0.03, rotation_thresh=np.deg2rad(30.0))

        self.assertEqual(len(group), 3)
        np.testing.assert_allclose(group.scores, [0.90, 0.70, 0.60], atol=1e-6)

    def test_point_sampling_is_repeatable_for_the_same_frame(self):
        first = server_module._sample_indices(100, 40, seed=7)
        second = server_module._sample_indices(100, 40, seed=7)
        different = server_module._sample_indices(100, 40, seed=8)

        np.testing.assert_array_equal(first, second)
        self.assertFalse(np.array_equal(first, different))

    def test_fallback_grasp_group_handles_empty_predictions(self):
        group = server_module.FallbackGraspGroup([])

        self.assertEqual(len(group), 0)
        self.assertEqual(group.translations.shape, (0, 3))
        self.assertEqual(group.rotation_matrices.shape, (0, 3, 3))
        self.assertEqual(group.widths.shape, (0,))
        self.assertIs(group.nms(), group)
        self.assertIs(group.sort_by_score(), group)

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
