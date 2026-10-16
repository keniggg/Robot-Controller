#!/usr/bin/env python3
import importlib.util
import pathlib
import sys
import types
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]
NODE_SCRIPT = ROOT / 'scripts' / 'tactile_skin_node.py'
WRAPPER_SCRIPT = ROOT / 'src' / 'alicia_flexible_grasp' / 'tactile' / 'tactile_sdk_wrapper.py'


def load_node_module():
    spec = importlib.util.spec_from_file_location('tactile_skin_node', str(NODE_SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_wrapper_module():
    spec = importlib.util.spec_from_file_location('tactile_sdk_wrapper', str(WRAPPER_SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeConfig:
    def __init__(self, sdk):
        self.sdk = sdk

    def set_pressure_value_type(self, value_type):
        self.sdk.configured.append(('type', self.sdk._addr_ref[0], value_type))

    def trigger_dynamic_zero(self):
        address = self.sdk._addr_ref[0]
        self.sdk.configured.append(('zero', address, None))
        self.sdk._zero_offsets[:] = [float(address * 100)] * 60


class FakePressure:
    def __init__(self, sdk):
        self.sdk = sdk

    def read_fast(self):
        addr = self.sdk._addr_ref[0]
        raw = [float(addr * 100 + addr)] * 60
        if not self.sdk._zero_offsets:
            return raw
        return [max(0.0, value - offset) for value, offset in zip(raw, self.sdk._zero_offsets)]


class FakeSDK:
    instances = []

    def __init__(self, port, slave_address=1, baudrate=4000000):
        self.port = port
        self.baudrate = baudrate
        self._addr_ref = [slave_address]
        self._zero_offsets = []
        self.configured = []
        self.config = FakeConfig(self)
        self.pressure = FakePressure(self)
        FakeSDK.instances.append(self)

    def connect(self):
        return True

    def disconnect(self):
        return True


class TactileDualSlaveTest(unittest.TestCase):
    def test_resolve_tactile_config_enables_dual_slave_addresses(self):
        module = load_node_module()

        resolved = module.resolve_tactile_config({
            'slave_address': 1,
            'left_slave_address': 2,
            'right_slave_address': 1,
        })

        self.assertTrue(resolved['dual_slave_addresses'])
        self.assertEqual(resolved['left_slave_address'], 2)
        self.assertEqual(resolved['right_slave_address'], 1)
        self.assertEqual(resolved['slave_addresses'], [2, 1])

    def test_wrapper_reads_each_configured_slave_address(self):
        fake_module = types.ModuleType('tactile_sdk')
        fake_module.TactilePressureSDK = FakeSDK
        original = sys.modules.get('tactile_sdk')
        sys.modules['tactile_sdk'] = fake_module
        try:
            FakeSDK.instances = []
            module = load_wrapper_module()
            wrapper = module.TactileSDKWrapper(
                port='/dev/test_skin',
                slave_address=1,
                slave_addresses=[2, 1],
                dynamic_zero_on_start=True,
            )

            wrapper.connect()
            left = wrapper.read_values(address=2)
            right = wrapper.read_values(address=1)
        finally:
            if original is None:
                sys.modules.pop('tactile_sdk', None)
            else:
                sys.modules['tactile_sdk'] = original

        self.assertEqual(left, [2.0] * 60)
        self.assertEqual(right, [1.0] * 60)
        self.assertEqual(
            FakeSDK.instances[0].configured,
            [('type', 2, 1), ('zero', 2, None), ('type', 1, 1), ('zero', 1, None)],
        )


if __name__ == '__main__':
    unittest.main()
