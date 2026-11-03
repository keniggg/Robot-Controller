"""Offline bootstrap scope/geometry/feedback contract tests; no ROS calls."""
from collections import deque
import importlib.util
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

PATH = Path(__file__).with_name('bounded_joint_response_diagnostic.py')
spec = importlib.util.spec_from_file_location('bootstrap_bound_diagnostic', PATH)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)


@pytest.mark.parametrize('extra,expected', [([], 4.8), (['--bootstrap-sync-bound-deg','2'], 2.), (['--bootstrap-sync-bound-deg','4.8'], 4.8)])
def test_explicit_bootstrap_bound_and_original_default(extra, expected):
    args = mod.parse_args(['--execute','--bootstrap-pending','--joint','5','--step-deg','2']+extra)
    assert args.bootstrap_sync_bound_deg == expected


@pytest.mark.parametrize('argv', [
    ['--bootstrap-sync-bound-deg','2'],
    ['--execute','--bootstrap-sync-bound-deg','2'],
    ['--bootstrap-pending','--bootstrap-sync-bound-deg','2'],
    ['--execute','--bootstrap-pending','--joint','2','--bootstrap-sync-bound-deg','2'],
    ['--execute','--bootstrap-pending','--step-deg','3','--bootstrap-sync-bound-deg','2'],
    ['--execute','--bootstrap-pending','--bootstrap-sync-bound-deg','1.9'],
    ['--execute','--bootstrap-pending','--bootstrap-sync-bound-deg','3'],
    ['--execute','--bootstrap-pending','--bootstrap-sync-bound-deg','nan'],
    ['--execute','--bootstrap-pending','--bootstrap-sync-bound-deg','inf'],
])
def test_bound_option_never_grants_implicit_bootstrap_or_wider_arbitrary_bound(argv):
    with pytest.raises(SystemExit): mod.parse_args(argv)


@pytest.mark.parametrize('bound', [2., 4.8])
def test_total_l1_bound_rejects_multi_axis_escape(bound):
    origin = np.zeros(7)
    current = origin.copy(); current[:6] = math.radians(bound/6.)
    mod.validate_bootstrap_sync_response(current, origin, bound)
    current[0] += math.radians(.001)
    assert np.max(np.abs(current[:6])) < math.radians(bound)
    with pytest.raises(RuntimeError, match='total joint response bound'):
        mod.validate_bootstrap_sync_response(current, origin, bound)


@pytest.mark.parametrize('bound', [2., 4.8])
def test_bootstrap_passes_selected_bound_to_all_six_geometry_reservations(monkeypatch, bound):
    obj = mod.Diagnostic.__new__(mod.Diagnostic)
    obj.args = SimpleNamespace(bootstrap_sync_bound_deg=bound)
    obj.guard = lambda: None
    obj.stationary = lambda stamp: {}
    obj.started_ns = obj.epoch = 1
    obj.fault = None
    obj.state = {'status': 'PENDING:POSITIVE_ENABLE_REQUESTED'}
    obj.reference = deque()
    m = mod.JointState(name=mod.NAMES, position=[0.]*6+[.04975])
    obj.accepted = deque([m])
    obj.snapshot = lambda label: 'fresh geometry'
    obj.fk, obj.gripper, obj.lo, obj.hi = object(), object(), object(), object()
    mixed_calls = []
    def mixed(*args):
        mixed_calls.append(args)
        return {'continuous': True}
    monkeypatch.setattr(mod, 'mixed_sync_geometry', mixed)
    calls = []
    def path(measured, target, geometry, **kwargs):
        calls.append((measured.copy(), target.copy(), geometry, kwargs))
        return {}
    obj.path = path
    class EndPreparation(Exception): pass
    def no_hardware(): raise EndPreparation()
    obj.driver_connection = no_hardware
    with pytest.raises(EndPreparation): obj.bootstrap_once()
    assert len(calls) == 1
    assert len(mixed_calls) == 1 and mixed_calls[0][-1] == bound
    assert calls[0][2] == 'fresh geometry'
    assert calls[0][3]['reserve_degrees'] == bound
    assert list(calls[0][3]['reserve_indices']) == list(range(6))


@pytest.mark.parametrize('bound', [2., 4.8])
def test_each_sync_feedback_latches_transient_total_escape(monkeypatch, bound):
    obj = mod.Diagnostic.__new__(mod.Diagnostic)
    obj.args = SimpleNamespace(bootstrap_sync_bound_deg=bound)
    obj.lock = threading.RLock(); obj.pair_changed = threading.Condition(obj.lock)
    obj.closed = False; obj.fault = None; obj.state = {}; obj.accepted = deque()
    obj.bootstrap_sync_origin = np.array([0.]*6+[.04975])
    obj.write_bag = lambda *args: None
    monkeypatch.setattr(mod.rospy.Time, 'now', lambda: mod.rospy.Time(10, 0))
    q = obj.bootstrap_sync_origin.copy(); q[:2] += math.radians((bound+.01)/2.)
    obj.receive('/alicia_d/accepted_joint_states', mod.JointState(name=mod.NAMES, position=q.tolist()))
    assert 'total joint response bound' in obj.fault
    obj.receive('/alicia_d/accepted_joint_states', mod.JointState(name=mod.NAMES, position=obj.bootstrap_sync_origin.tolist()))
    assert 'total joint response bound' in obj.fault


@pytest.mark.parametrize('bound', [2., 4.8])
def test_real_geometry_reservation_and_report_match_selected_radius(bound):
    # Archived URDF plus synthetic fixed board; the MoveIt service is an offline stub.
    from test_bounded_joint3_response_diagnostic import offline_path_fixture
    obj, q, geometry, checked = offline_path_fixture()
    obj.args = SimpleNamespace(joint=5)
    details = mod.Diagnostic.path(obj, q, q, geometry,
                                  reserve_indices=range(6), reserve_degrees=bound)
    values = np.asarray(checked)
    assert len(checked) == 17*13
    assert details['response_reserved_degrees'] == bound
    assert np.allclose(np.min(values[:,:6], axis=0), q[:6]-math.radians(bound), rtol=0, atol=1e-14)
    assert np.allclose(np.max(values[:,:6], axis=0), q[:6]+math.radians(bound), rtol=0, atol=1e-14)
    assert np.all(values[:,6] == q[6])
    assert details['minimum_plane_clearance_m'] >= .04
    assert details['minimum_full_board_edge_px'] >= 20
