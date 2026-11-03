"""Post-action evidence persistence is finite and preserves actual raw bytes."""
from dataclasses import dataclass
import hashlib
import json
from types import SimpleNamespace as NS
from pathlib import Path
import sys

import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools/tests'))
from storage_os_adapter import storage_os, tmp_path

from alicia_flexible_grasp.robot import rgb_acquisition_artifacts as module


def test_missing_external_mount_has_no_fallback(storage_os):
    storage_os.states['ssd']['mounted'] = False
    with pytest.raises(RuntimeError,match='no fallback'):
        module.require_storage()


@pytest.mark.parametrize('backend', ['ssd','sanstand'])
def test_saved_raw_pixels_and_exact_numpy_metadata_are_replayable(tmp_path,storage_os,monkeypatch,backend):
    monkeypatch.setenv(storage_os.helper.BACKEND_ENV,backend)
    selected = storage_os.select(backend)
    @dataclass
    class Volume:
        T_base_camera: object
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
    assert path.parent == selected.ros_log_root
    assert report['storage'] == selected.validate(path)
    assert set(report['frames'])=={'initial'}
    assert report['outcome']['value']==10**18
    with np.load(path/'initial_source.npz') as data:
        assert data['depth_raw'].dtype==np.uint16
        assert data['depth_raw'].tobytes()==raw.tobytes()
        assert data['color_bgr'].tobytes()==rgb.tobytes()
        assert int(data['source_stamp_ns'])==10**18
    assert report['frames']['initial']['source_depth_sha256']==hashlib.sha256(raw.tobytes()).hexdigest()


def test_actual_serialized_trajectory_and_model_bytes_are_preserved(storage_os):
    payload = b'\x00real\xffserialized\x00trajectory'
    class Trajectory:
        def serialize(self, stream): stream.write(payload)
    model = '<robot name="fixture">\n</robot>\n'
    path = Path(module.save_artifacts({}, {'passed':False}, trajectory=Trajectory(), robot_description=model))
    report = json.loads((path/'result.json').read_bytes())
    assert (path/'trajectory.rosmsg').read_bytes() == payload
    assert report['trajectory_sha256'] == hashlib.sha256(payload).hexdigest()
    assert (path/'robot_description.urdf').read_bytes() == model.encode()
    assert report['outcome']['passed'] is False


@pytest.mark.parametrize('fault', ['unmounted','remounted','config_changed'])
def test_storage_change_during_serialization_prevents_evidence_publication(storage_os, fault):
    class Trajectory:
        def serialize(self, stream):
            stream.write(b'actual serialization before storage loss')
            if fault == 'unmounted': storage_os.states['ssd']['mounted'] = False
            elif fault == 'remounted': storage_os.states['ssd']['mount_id'] += 1
            else: storage_os.config.write_text(storage_os.config.read_text()+' ')
    with pytest.raises(RuntimeError): module.save_artifacts({}, {}, trajectory=Trajectory())
    assert not list(storage_os.selected.ros_log_root.rglob('trajectory.rosmsg'))
    assert not list(storage_os.selected.ros_log_root.rglob('result.json'))


def test_rgb_output_symlink_cannot_redirect_to_another_disk(storage_os):
    selected = storage_os.selected
    other = storage_os.select('sanstand')
    selected.ros_log_root.symlink_to(other.base, target_is_directory=True)
    with pytest.raises(RuntimeError, match='symlink'): module.save_artifacts({}, {})
    assert not list(other.base.iterdir())
