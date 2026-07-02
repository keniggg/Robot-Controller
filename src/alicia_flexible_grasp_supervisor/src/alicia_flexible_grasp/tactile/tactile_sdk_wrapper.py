import os
import sys
import time
import random

class TactileSDKWrapper:
    def __init__(self, port, slave_address=1, baudrate=4000000, sdk_path=None,
                 pressure_value_type=1, dynamic_zero_on_start=True, simulate=False):
        self.port = port
        self.slave_address = int(slave_address)
        self.baudrate = int(baudrate)
        self.sdk_path = os.path.expanduser(sdk_path) if sdk_path else None
        self.pressure_value_type = int(pressure_value_type)
        self.dynamic_zero_on_start = bool(dynamic_zero_on_start)
        self.simulate = bool(simulate)
        self.sdk = None
        self._phase = 0

    def connect(self):
        if self.simulate:
            return True
        if self.sdk_path and self.sdk_path not in sys.path:
            sys.path.insert(0, self.sdk_path)
        try:
            from tactile_sdk import TactilePressureSDK
            self.sdk = TactilePressureSDK(self.port, slave_address=self.slave_address, baudrate=self.baudrate)
            self.sdk.connect()
            try:
                self.sdk.config.set_pressure_value_type(self.pressure_value_type)
            except Exception:
                pass
            if self.dynamic_zero_on_start:
                try:
                    self.sdk.config.trigger_dynamic_zero()
                except Exception:
                    pass
            return True
        except Exception as exc:
            raise RuntimeError('Failed to connect tactile SDK: %s' % exc)

    def close(self):
        if self.sdk is not None:
            try:
                self.sdk.disconnect()
            except Exception:
                pass
            self.sdk = None

    def read_values(self):
        if self.simulate:
            return self._simulate_values()
        if self.sdk is None:
            return None
        try:
            values = self.sdk.pressure.read_fast()
            if values is not None:
                return [float(v) for v in values]
            return None
        except Exception:
            return None

    def zero(self):
        if self.sdk is not None:
            try:
                self.sdk.config.trigger_dynamic_zero()
                return True
            except Exception:
                return False
        return self.simulate

    def _simulate_values(self):
        self._phase += 1
        vals = [0.0] * 60
        # generate a soft moving contact blob on the first half
        idx = (self._phase // 5) % 30
        amp = 800.0 + 400.0 * (1 if (self._phase // 100) % 2 else 0)
        for k in range(30):
            d = abs(k - idx)
            vals[k] = max(0.0, amp - d * 180.0) + random.random() * 10.0
        # second half with lower pressure
        for k in range(30, 60):
            vals[k] = max(0.0, 500.0 - abs(k - 45) * 110.0) + random.random() * 8.0
        return vals
