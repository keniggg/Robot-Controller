#!/usr/bin/env python3
import importlib.util
import json
import pathlib
import sys
import threading
import unittest
import urllib.request


ROOT = pathlib.Path(__file__).resolve().parents[3]
SCRIPT = ROOT / 'tools' / 'mujoco_digital_twin_server.py'
spec = importlib.util.spec_from_file_location('mujoco_digital_twin_server', str(SCRIPT))
server_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server_module)


class MujocoDigitalTwinServerProtocolTest(unittest.TestCase):
    def test_mock_server_serves_health_sync_predict_and_simulate(self):
        server = server_module.make_server(
            '127.0.0.1',
            0,
            grasp_backend=server_module.MockGraspNetBackend(),
            sim_backend=server_module.MockDigitalTwinBackend(),
        )
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            base_url = 'http://127.0.0.1:%d' % server.server_port
            health = self._json_get(base_url + '/health')
            self.assertTrue(health['ok'])
            self.assertEqual(health['grasp_backend']['backend'], 'mock')
            self.assertEqual(health['digital_twin']['backend'], 'mock_mujoco')

            sync = self._json_post(
                base_url + '/sync_joint_state',
                {'joint_names': ['Joint1'], 'joint_positions': [0.1]},
            )
            self.assertTrue(sync['ok'])

            predict = self._json_post(
                base_url + '/predict',
                {'encoding': 'mock', 'max_candidates': 1},
            )
            self.assertTrue(predict['ok'])
            self.assertEqual(len(predict['candidates']), 1)

            sim = self._json_post(
                base_url + '/simulate_grasp',
                {
                    'grasp_sequence_base': [
                        {'name': 'pregrasp', 'position': [0.1, 0.0, 0.2], 'quaternion_xyzw': [0, 0, 0, 1]},
                        {'name': 'approach', 'position': [0.2, 0.0, 0.2], 'quaternion_xyzw': [0, 0, 0, 1]},
                        {'name': 'grasp', 'position': [0.3, 0.0, 0.2], 'quaternion_xyzw': [0, 0, 0, 1]},
                        {'name': 'lift', 'position': [0.3, 0.0, 0.25], 'quaternion_xyzw': [0, 0, 0, 1]},
                    ],
                },
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

        self.assertTrue(sim['ok'])
        self.assertTrue(sim['simulation_ok'])
        self.assertGreaterEqual(sim['score'], 80)
        self.assertIn('diagnosis', sim)

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
