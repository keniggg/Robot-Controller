#!/usr/bin/env python3
import pathlib
import sys
import unittest
from unittest import mock

import numpy as np


ROOT = pathlib.Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / 'src'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from alicia_flexible_grasp.vision.grasp6d_adapter import CameraIntrinsics
from alicia_flexible_grasp.vision import remote_grasp6d_client as client_module
from alicia_flexible_grasp.vision.remote_grasp6d_client import (
    CandidateContractError,
    GRASP6D_CANDIDATE_FIELDS,
    GRASP6D_PROTOCOL_VERSION,
    RemoteGrasp6DClient,
    RemoteGraspCandidate,
    RemotePredictionBundle,
    decode_remote_grasp_response,
    decode_rgbd_payload,
    encode_rgbd_payload,
    validate_remote_grasp6d_url,
)


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


class RemoteGrasp6DClientTest(unittest.TestCase):
    @staticmethod
    def _predict_with_response(response):
        client = RemoteGrasp6DClient('http://127.0.0.1:8000')
        client._request_json = lambda path, payload: response
        intrinsics = CameraIntrinsics(
            width=2,
            height=2,
            fx=100.0,
            fy=100.0,
            cx=0.5,
            cy=0.5,
            depth_scale=0.001,
        )
        result = client.predict(
            np.zeros((2, 2, 3), dtype=np.uint8),
            np.full((2, 2), 200, dtype=np.uint16),
            intrinsics,
            request_id=41,
            snapshot_stamp_sec=123.25,
        )
        return client, result

    @staticmethod
    def _protocol_response(ok=True, request_id=41, snapshot_stamp_sec=123.25):
        response = {
            'ok': bool(ok),
            'protocol_version': GRASP6D_PROTOCOL_VERSION,
            'candidate_fields': list(GRASP6D_CANDIDATE_FIELDS),
            'request_id': request_id,
            'snapshot_stamp_sec': snapshot_stamp_sec,
            'candidates': [],
            'diagnostics': {'contract': 'v3'},
            'server_receive_sec': 200.0,
            'server_send_sec': 200.25,
            'preprocess_ms': 1.0,
            'inference_ms': 2.0,
            'postprocess_ms': 3.0,
            'server_total_ms': 6.0,
            'gpu_allocated_mb': 100.0,
            'gpu_reserved_mb': 120.0,
            'gpu_peak_allocated_mb': 110.0,
        }
        if not ok:
            response['error'] = 'checkpoint missing'
        return response

    def test_rgbd_payload_round_trip_preserves_arrays_and_intrinsics(self):
        color = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        depth = np.array([[1000, 1001, 0], [1200, 1300, 1400]], dtype=np.uint16)
        intrinsics = CameraIntrinsics(width=3, height=2, fx=438.6, fy=438.4, cx=1.2, cy=0.7, depth_scale=0.0001)

        payload = encode_rgbd_payload(
            color,
            depth,
            intrinsics,
            request_id=7,
            snapshot_stamp_sec=12.25,
            frame_id='camera_link',
            stamp_sec=12.25,
            max_candidates=8,
        )
        decoded = decode_rgbd_payload(payload)

        self.assertEqual(decoded['frame_id'], 'camera_link')
        self.assertAlmostEqual(decoded['stamp_sec'], 12.25)
        self.assertEqual(decoded['request_id'], 7)
        self.assertAlmostEqual(decoded['snapshot_stamp_sec'], 12.25)
        self.assertEqual(decoded['max_candidates'], 8)
        self.assertEqual(decoded['intrinsics']['depth_scale'], 0.0001)
        np.testing.assert_array_equal(decoded['color_bgr'], color)
        np.testing.assert_array_equal(decoded['depth_raw'], depth)

    def test_response_accepts_rotation_matrix_and_normalizes_quaternion(self):
        response = {
            'ok': True,
            'candidates': [
                {
                    'score': 0.91,
                    'width_m': 0.045,
                    'height_m': 0.02,
                    'depth_m': 0.03,
                    'translation_m': [0.1, 0.2, 0.3],
                    'rotation_matrix': [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0],
                    ],
                }
            ],
        }

        candidates = decode_remote_grasp_response(response)

        self.assertEqual(len(candidates), 1)
        self.assertAlmostEqual(candidates[0].score, 0.91)
        self.assertAlmostEqual(candidates[0].width_m, 0.045)
        self.assertAlmostEqual(candidates[0].height_m, 0.02)
        self.assertAlmostEqual(candidates[0].depth_m, 0.03)
        np.testing.assert_allclose(candidates[0].translation_m, [0.1, 0.2, 0.3])
        np.testing.assert_allclose(candidates[0].quaternion_xyzw, [0.0, 0.0, 0.0, 1.0])

    def test_response_rejects_invalid_rotation_matrix_contract(self):
        reflection = np.eye(3, dtype=float)
        reflection[2, 2] = -1.0
        nonfinite = np.eye(3, dtype=float)
        nonfinite[0, 1] = np.nan
        for name, rotation in (
            ('zero', np.zeros((3, 3), dtype=float)),
            ('reflection', reflection),
            ('scaled', 2.0 * np.eye(3, dtype=float)),
            ('nan', nonfinite),
        ):
            with self.subTest(name=name):
                with self.assertRaises(CandidateContractError) as context:
                    decode_remote_grasp_response(
                        {
                            'ok': True,
                            'candidates': [
                                {
                                    'score': 0.8,
                                    'depth_m': 0.03,
                                    'translation_m': [0.1, 0.0, 0.2],
                                    'rotation_matrix': rotation.tolist(),
                                }
                            ],
                        }
                    )
                self.assertEqual(context.exception.code, 'ORIENTATION_INVALID')

    def test_response_rejects_nonfinite_or_zero_direct_quaternion(self):
        for name, quaternion in (
            ('zero', [0.0, 0.0, 0.0, 0.0]),
            ('nan', [0.0, 0.0, float('nan'), 1.0]),
            ('inf', [0.0, float('inf'), 0.0, 1.0]),
        ):
            with self.subTest(name=name):
                with self.assertRaises(CandidateContractError) as context:
                    decode_remote_grasp_response(
                        {
                            'ok': True,
                            'candidates': [
                                {
                                    'score': 0.8,
                                    'depth_m': 0.03,
                                    'translation_m': [0.1, 0.0, 0.2],
                                    'quaternion_xyzw': quaternion,
                                }
                            ],
                        }
                    )
                self.assertEqual(context.exception.code, 'ORIENTATION_INVALID')

    def test_response_marks_old_server_candidate_depth_as_missing(self):
        candidates = decode_remote_grasp_response(
            {
                'ok': True,
                'candidates': [
                    {
                        'score': 0.8,
                        'translation_m': [0.1, 0.0, 0.2],
                        'rotation_matrix': np.eye(3).tolist(),
                    }
                ],
            },
            require_candidate_depth=False,
        )

        self.assertIsNone(candidates[0].depth_m)
        self.assertIsNone(candidates[0].height_m)
        self.assertIsNone(candidates[0].tool0_translation_m)

    def test_strict_response_rejects_missing_depth(self):
        with self.assertRaises(CandidateContractError) as context:
            decode_remote_grasp_response(
                {
                    'ok': True,
                    'candidates': [
                        {
                            'score': 0.8,
                            'translation_m': [0.1, 0.0, 0.2],
                            'rotation_matrix': np.eye(3).tolist(),
                        }
                    ],
                }
            )
        self.assertEqual(context.exception.code, 'DEPTH_MISSING')

    def test_response_rejects_nonfinite_and_out_of_domain_depth(self):
        for depth, code in (
            (float('nan'), 'DEPTH_INVALID'),
            (float('inf'), 'DEPTH_INVALID'),
            (False, 'DEPTH_INVALID'),
            (0.001, 'DEPTH_OUT_OF_RANGE'),
            (0.50, 'DEPTH_OUT_OF_RANGE'),
        ):
            with self.subTest(depth=depth):
                with self.assertRaises(CandidateContractError) as context:
                    decode_remote_grasp_response(
                        {
                            'ok': True,
                            'candidates': [
                                {
                                    'score': 0.8,
                                    'depth_m': depth,
                                    'translation_m': [0.1, 0.0, 0.2],
                                    'rotation_matrix': np.eye(3).tolist(),
                                }
                            ],
                        }
                    )
                self.assertEqual(context.exception.code, code)

    def test_response_rejects_backend_error(self):
        with self.assertRaisesRegex(RuntimeError, 'checkpoint missing'):
            decode_remote_grasp_response({'ok': False, 'error': 'checkpoint missing'})

    def test_predict_accepts_exact_unified_success_protocol_envelope(self):
        client, candidates = self._predict_with_response(
            self._protocol_response(ok=True)
        )

        self.assertEqual(GRASP6D_PROTOCOL_VERSION, 3)
        self.assertEqual(candidates, [])
        self.assertEqual(client.last_diagnostics, {'contract': 'v3'})
        self.assertEqual(
            client.last_performance,
            {
                field: self._protocol_response()[field]
                for field in PERFORMANCE_FIELDS
            },
        )

    def test_predict_bundle_is_immutable_and_does_not_touch_legacy_state(self):
        client = RemoteGrasp6DClient('http://127.0.0.1:8000')
        client.last_diagnostics = {'legacy': 'keep'}
        client.last_performance = {'server_total_ms': 999.0}
        client._request_json = lambda _path, _payload: self._protocol_response()
        client._timing_clock = iter(
            (0.000, 0.010, 0.010, 0.030, 0.030, 0.037)
        ).__next__
        intrinsics = CameraIntrinsics(
            width=2,
            height=2,
            fx=100.0,
            fy=100.0,
            cx=0.5,
            cy=0.5,
            depth_scale=0.001,
        )

        bundle = client.predict_bundle(
            np.zeros((2, 2, 3), dtype=np.uint8),
            np.full((2, 2), 200, dtype=np.uint16),
            intrinsics,
            request_id=41,
            snapshot_stamp_sec=123.25,
        )

        self.assertIsInstance(bundle, RemotePredictionBundle)
        self.assertEqual(bundle.request_id, 41)
        self.assertEqual(bundle.candidates, ())
        self.assertEqual(bundle.diagnostics, {'contract': 'v3'})
        self.assertAlmostEqual(bundle.encode_ms, 10.0)
        self.assertAlmostEqual(bundle.transport_ms, 20.0)
        self.assertAlmostEqual(bundle.decode_ms, 7.0)
        with self.assertRaises(TypeError):
            bundle.diagnostics['mutated'] = True
        with self.assertRaises(AttributeError):
            bundle.transport_ms = 99.0
        self.assertEqual(client.last_diagnostics, {'legacy': 'keep'})
        self.assertEqual(
            client.last_performance,
            {'server_total_ms': 999.0},
        )

    def test_prediction_bundle_defensively_copies_candidate_payloads(self):
        source = RemoteGraspCandidate(
            score=0.8,
            translation_m=np.array([0.1, 0.0, 0.2]),
            quaternion_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
            width_m=0.04,
            depth_m=0.03,
        )

        bundle = RemotePredictionBundle(
            request_id=1,
            snapshot_stamp_sec=10.0,
            candidates=(source,),
            diagnostics={},
            performance={},
            encode_ms=0.0,
            transport_ms=0.0,
            decode_ms=0.0,
        )
        source.score = 0.1
        source.translation_m[0] = 9.0

        self.assertEqual(bundle.candidates[0].score, 0.8)
        np.testing.assert_allclose(
            bundle.candidates[0].translation_m,
            [0.1, 0.0, 0.2],
        )
        with self.assertRaises(AttributeError):
            bundle.candidates[0].score = 0.2
        with self.assertRaises(ValueError):
            bundle.candidates[0].translation_m[0] = 0.2

    def test_legacy_predict_returns_mutable_candidate_copy(self):
        response = self._protocol_response()
        response['candidates'] = [
            {
                'score': 0.8,
                'width_m': 0.04,
                'height_m': 0.02,
                'depth_m': 0.03,
                'translation_m': [0.1, 0.0, 0.2],
                'rotation_matrix': np.eye(3).tolist(),
            }
        ]

        _client, candidates = self._predict_with_response(response)
        candidate = candidates[0]
        candidate.score = 0.2
        candidate.translation_m[0] = 0.3

        self.assertIsInstance(candidate, RemoteGraspCandidate)
        self.assertEqual(candidate.score, 0.2)
        self.assertEqual(candidate.translation_m[0], 0.3)

    def test_predict_requires_exact_protocol3_request_correlation(self):
        response = self._protocol_response()
        response['candidates'] = [
            {
                'score': 0.9,
                'width_m': 0.04,
                'height_m': 0.02,
                'depth_m': 0.03,
                'translation_m': [0.1, 0.2, 0.3],
                'rotation_matrix': np.eye(3).tolist(),
            }
        ]

        client, candidates = self._predict_with_response(response)

        self.assertEqual(len(candidates), 1)
        self.assertGreaterEqual(client.last_performance['server_total_ms'], 0.0)

    def test_predict_drops_mismatched_request_or_stamp(self):
        for request_id, snapshot_stamp_sec in (
            (42, 123.25),
            (41, 123.5),
        ):
            with self.subTest(
                request_id=request_id,
                snapshot_stamp_sec=snapshot_stamp_sec,
            ):
                response = self._protocol_response(
                    request_id=request_id,
                    snapshot_stamp_sec=snapshot_stamp_sec,
                )
                with self.assertRaisesRegex(ValueError, 'correlation'):
                    self._predict_with_response(response)

    def test_predict_rejects_nonboolean_ok_and_malformed_envelope_collections(self):
        response = self._protocol_response()
        response['ok'] = 'false'
        response.pop('candidates')
        with self.assertRaisesRegex(ValueError, 'ok'):
            self._predict_with_response(response)

        for ok in (True, False):
            for label, field, value in (
                ('missing-candidates', 'candidates', None),
                ('tuple-candidates', 'candidates', ()),
                ('nondict-candidate', 'candidates', [[]]),
                ('missing-diagnostics', 'diagnostics', None),
                ('list-diagnostics', 'diagnostics', []),
            ):
                with self.subTest(ok=ok, case=label):
                    response = self._protocol_response(ok=ok)
                    if label.startswith('missing-'):
                        response.pop(field)
                    else:
                        response[field] = value
                    with self.assertRaisesRegex(ValueError, field):
                        self._predict_with_response(response)

    def test_predict_rejects_missing_or_invalid_mandatory_performance(self):
        for field in PERFORMANCE_FIELDS:
            for label, value in (
                ('missing', None),
                ('bool', True),
                ('numeric-string', '1.0'),
                ('negative', -0.1),
                ('nonfinite', float('nan')),
            ):
                with self.subTest(field=field, value=label):
                    response = self._protocol_response()
                    if label == 'missing':
                        response.pop(field)
                    else:
                        response[field] = value
                    with self.assertRaisesRegex(ValueError, field):
                        self._predict_with_response(response)

    def test_payload_rejects_invalid_request_correlation(self):
        intrinsics = CameraIntrinsics(
            width=2,
            height=2,
            fx=100.0,
            fy=100.0,
            cx=0.5,
            cy=0.5,
            depth_scale=0.001,
        )
        for request_id, snapshot_stamp_sec in (
            (True, 1.0),
            (0, 1.0),
            (-1, 1.0),
            (1, 0.0),
            (1, -1.0),
            (1, float('nan')),
            (1, float('inf')),
        ):
            with self.subTest(
                request_id=request_id,
                snapshot_stamp_sec=snapshot_stamp_sec,
            ):
                with self.assertRaisesRegex(ValueError, 'request_id|snapshot_stamp_sec'):
                    encode_rgbd_payload(
                        np.zeros((2, 2, 3), dtype=np.uint8),
                        np.full((2, 2), 200, dtype=np.uint16),
                        intrinsics,
                        request_id=request_id,
                        snapshot_stamp_sec=snapshot_stamp_sec,
                    )

    def test_predict_validates_exact_protocol_before_reporting_backend_failure(self):
        with self.assertRaisesRegex(RuntimeError, 'checkpoint missing'):
            self._predict_with_response(self._protocol_response(ok=False))

    def test_predict_clears_stale_diagnostics_before_transport_failure(self):
        client = RemoteGrasp6DClient('http://127.0.0.1:8000')
        client.last_diagnostics = {'stale': 'previous inference'}
        client.last_performance = {'server_total_ms': 999.0}

        def fail_request(_path, _payload):
            raise RuntimeError('transport unavailable')

        client._request_json = fail_request
        intrinsics = CameraIntrinsics(
            width=2,
            height=2,
            fx=100.0,
            fy=100.0,
            cx=0.5,
            cy=0.5,
            depth_scale=0.001,
        )

        with self.assertRaisesRegex(RuntimeError, 'transport unavailable'):
            client.predict(
                np.zeros((2, 2, 3), dtype=np.uint8),
                np.full((2, 2), 200, dtype=np.uint16),
                intrinsics,
                request_id=41,
                snapshot_stamp_sec=123.25,
            )

        self.assertEqual(client.last_diagnostics, {})
        self.assertEqual(client.last_performance, {})

    def test_predict_keeps_state_empty_when_candidate_decode_fails(self):
        client = RemoteGrasp6DClient('http://127.0.0.1:8000')
        client.last_diagnostics = {'stale': 'previous inference'}
        client.last_performance = {'server_total_ms': 999.0}
        response = self._protocol_response()
        response['candidates'] = [
            {
                'score': 0.9,
                'width_m': 0.04,
                'height_m': 0.02,
                'translation_m': [0.1, 0.2, 0.3],
                'rotation_matrix': np.eye(3).tolist(),
            }
        ]
        client._request_json = lambda _path, _payload: response
        intrinsics = CameraIntrinsics(
            width=2,
            height=2,
            fx=100.0,
            fy=100.0,
            cx=0.5,
            cy=0.5,
            depth_scale=0.001,
        )

        with self.assertRaises(CandidateContractError) as context:
            client.predict(
                np.zeros((2, 2, 3), dtype=np.uint8),
                np.full((2, 2), 200, dtype=np.uint16),
                intrinsics,
                request_id=41,
                snapshot_stamp_sec=123.25,
            )

        self.assertEqual(context.exception.code, 'DEPTH_MISSING')
        self.assertEqual(client.last_diagnostics, {})
        self.assertEqual(client.last_performance, {})

    def test_predict_rejects_nonfinite_value_hidden_outside_candidate_fields(self):
        for nonfinite in (float('nan'), float('inf'), float('-inf')):
            with self.subTest(nonfinite=nonfinite):
                response = self._protocol_response(ok=True)
                response['unexpected_diagnostic'] = nonfinite
                with self.assertRaisesRegex(ValueError, 'strict-JSON'):
                    self._predict_with_response(response)

    def test_http_decoder_rejects_nonstandard_json_constants(self):
        class Response:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            @staticmethod
            def read():
                return b'{"ok":true,"unexpected":NaN}'

        client = RemoteGrasp6DClient('http://127.0.0.1:8000')
        with mock.patch.object(
            client_module.urllib.request,
            'urlopen',
            return_value=Response(),
        ):
            with self.assertRaisesRegex(ValueError, 'non-standard JSON'):
                client._request_json('/predict', None)

    def test_predict_rejects_missing_or_wrong_protocol_version_for_success_and_failure(self):
        for ok in (True, False):
            for label, version in (
                ('missing', None),
                ('old', 1),
                ('string', '3'),
                ('float', 3.0),
            ):
                with self.subTest(ok=ok, version=label):
                    response = self._protocol_response(ok=ok)
                    if label == 'missing':
                        response.pop('protocol_version')
                    else:
                        response['protocol_version'] = version

                    with self.assertRaisesRegex(ValueError, 'protocol_version'):
                        self._predict_with_response(response)

    def test_predict_rejects_nonexact_candidate_fields_for_success_and_failure(self):
        expected = list(GRASP6D_CANDIDATE_FIELDS)
        invalid_fields = (
            ('missing', None),
            ('tuple', tuple(expected)),
            ('field-missing', expected[:-1]),
            ('field-extra', expected + ['quaternion_xyzw']),
            ('reordered', list(reversed(expected))),
        )
        for ok in (True, False):
            for label, fields in invalid_fields:
                with self.subTest(ok=ok, fields=label):
                    response = self._protocol_response(ok=ok)
                    if label == 'missing':
                        response.pop('candidate_fields')
                    else:
                        response['candidate_fields'] = fields

                    with self.assertRaisesRegex(ValueError, 'candidate_fields'):
                        self._predict_with_response(response)

    def test_remote_url_validation_rejects_placeholder_angle_brackets(self):
        with self.assertRaisesRegex(ValueError, 'placeholder'):
            validate_remote_grasp6d_url('http://<WSL或Windows可访问IP>:8000')

    def test_remote_url_validation_rejects_missing_http_scheme(self):
        with self.assertRaisesRegex(ValueError, 'http'):
            RemoteGrasp6DClient('192.168.26.1:8000')

    def test_remote_url_validation_normalizes_trailing_slash(self):
        self.assertEqual(validate_remote_grasp6d_url('http://192.168.26.1:8000/'), 'http://192.168.26.1:8000')


if __name__ == '__main__':
    unittest.main()
