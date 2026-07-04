#!/usr/bin/env python3
import importlib.util
import json
import pathlib
import sys
import threading
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

    def _json_get(self, url):
        with urllib.request.urlopen(url, timeout=2.0) as response:
            return json.loads(response.read().decode('utf-8'))

    def _json_post(self, url, payload):
        data = json.dumps(payload).encode('utf-8')
        request = urllib.request.Request(url, data=data, headers={'Content-Type': 'application/json'}, method='POST')
        with urllib.request.urlopen(request, timeout=2.0) as response:
            return json.loads(response.read().decode('utf-8'))


if __name__ == '__main__':
    unittest.main()
