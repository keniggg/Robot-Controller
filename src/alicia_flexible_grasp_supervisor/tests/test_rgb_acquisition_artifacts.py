"""Post-action evidence persistence is finite and preserves actual raw bytes."""
from dataclasses import dataclass
import hashlib
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_flexible_grasp.robot import rgb_acquisition_artifacts as module


def test_missing_external_mount_has_no_fallback(monkeypatch):
    monkeypatch.setattr(module,'MOUNT',NS(is_mount=lambda:False))
    with pytest.raises(RuntimeError,match='no system-disk fallback'):
        module.require_storage()


def test_saved_raw_pixels_and_exact_numpy_metadata_are_replayable(tmp_path,monkeypatch):
    @dataclass
    class Volume:
        T_base_camera: object
    monkeypatch.setattr(module,'require_storage',lambda:tmp_path)
    rgb=np.arange(18,dtype=np.uint8).reshape(2,3,3)
    raw=np.array([[0,31,1000],[111,200,60000]],dtype=np.uint16)
    context=dict(camera=dict(fx=100.,fy=100.,cx=1.,cy=1.),tool_camera=np.eye(4).tolist())
    source=dict(rgb=rgb,depth_raw=raw,depth_scale=.001,mask=np.ones((2,3),dtype=bool),
        volume=Volume(np.eye(4)),source_context=context,source_stamp_ns=10**18,
        source_id='original',source_rgb_sha256=hashlib.sha256(rgb.tobytes()).hexdigest(),
        source_depth_sha256=hashlib.sha256(raw.tobytes()).hexdigest(),q_source=[0.]*6,
        interest=dict(kind='observation_only'),accepted_before=(10**18-1,np.zeros(6)),
        accepted_after=(10**18+1,np.ones(6)))
    path=module.save_artifacts(dict(initial=source,unbounded_extra=source),
                              dict(value=np.int64(10**18)),audit=dict(clearance=np.float64(.004)))
    from pathlib import Path
    path=Path(path)
    report=json.loads((path/'result.json').read_text())
    assert set(report['frames'])=={'initial'}
    assert report['outcome']['value']==10**18
    with np.load(path/'initial_source.npz') as data:
        assert data['depth_raw'].dtype==np.uint16
        assert data['depth_raw'].tobytes()==raw.tobytes()
        assert data['color_bgr'].tobytes()==rgb.tobytes()
        assert int(data['source_stamp_ns'])==10**18
    assert report['frames']['initial']['source_depth_sha256']==hashlib.sha256(raw.tobytes()).hexdigest()
