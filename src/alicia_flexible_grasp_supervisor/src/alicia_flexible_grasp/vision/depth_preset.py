"""Measured-depth preset, independent of the task's stationary-filter mode.

This selects an SDK matcher preset, not a calibration or a depth-image repair.
Invalid pixels remain invalid. A saved baseline survives camera process restart.
Unknown defaults to high_accuracy; saved_baseline is an explicit alternative.
"""
import hashlib
import json
from pathlib import Path


class ModeDepthPreset:
    def __init__(self, rs, device, config, depth_scale):
        unknown_preset = config.get('unknown_preset', 'high_accuracy')
        if type(unknown_preset) is not str or unknown_preset not in ('high_accuracy', 'saved_baseline'):
            raise ValueError('unsupported unknown depth preset: expected high_accuracy or saved_baseline')
        self.unknown_preset = unknown_preset
        path = Path(config['baseline_path']).expanduser()
        saved = json.loads(path.read_text())
        serial = str(device.get_info(rs.camera_info.serial_number))
        if saved.get('serial_number') != serial:
            raise ValueError('depth preset baseline belongs to another camera')
        baseline = saved.get('advanced_settings')
        if not isinstance(baseline, dict) or not isinstance(baseline.get('parameters'), dict):
            raise ValueError('depth preset baseline has no advanced parameters')
        self.baseline = json.dumps(baseline)
        self.baseline_hash = hashlib.sha256(self.baseline.encode()).hexdigest()
        self.rs = rs
        self.sensor = device.first_depth_sensor()
        self.advanced = rs.rs400_advanced_mode(device)
        self.depth_scale = float(depth_scale)
        self.mode = None
        self.dirty = False

    def _restore(self):
        self.advanced.load_json(self.baseline)
        actual = json.loads(self.advanced.serialize_json())['parameters']
        expected = json.loads(self.baseline)['parameters']
        if actual != expected:
            raise ValueError('depth preset baseline restoration mismatch')
        self._check_scale()
        self.dirty = False

    def _check_scale(self):
        if abs(float(self.sensor.get_depth_scale()) - self.depth_scale) > 1e-12:
            raise ValueError('depth preset changed measurement units')

    def apply(self, mode):
        if mode not in ('carton', 'unknown'):
            raise ValueError('invalid depth preset task mode')
        if mode == self.mode:
            return False
        # Restore before the preset, too: never inherit a partial or crashed
        # previous sensor configuration as the next task's baseline.
        self.dirty = True
        try:
            self._restore()
            if mode == 'unknown' and self.unknown_preset == 'high_accuracy':
                self.dirty = True
                self.sensor.set_option(self.rs.option.visual_preset, 3.)
                if float(self.sensor.get_option(self.rs.option.visual_preset)) != 3.:
                    raise ValueError('high_accuracy preset was not applied')
                self._check_scale()
            self.mode = mode
            return True
        except Exception:
            self.mode = None
            self._restore()
            raise

    def evidence(self):
        return dict(mode=self.mode,
                    preset=(self.unknown_preset if self.mode == 'unknown' else
                            'saved_baseline' if self.mode == 'carton' else None),
                    configured_unknown_preset=self.unknown_preset,
                    baseline_sha256=self.baseline_hash,
                    depth_scale_m_per_unit=self.depth_scale,
                    invalid_depth_filling=False)

    def close(self):
        if self.dirty:
            self._restore()
        self.mode = None
