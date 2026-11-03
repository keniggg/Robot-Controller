"""Pure replay inputs and external-only storage, with no ROS/device clients.

These tests prove serialization/recovery, not execution authority or a grasp.
The spline test runs the unchanged production quintic kernel on restored data.
"""
import ast
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]/'tools/tests'))
from storage_os_adapter import storage_os, tmp_path


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT/'src/alicia_flexible_grasp/robot/contact_replay_receipt.py'
SPEC = importlib.util.spec_from_file_location('gen38_contact_replay_receipt', HELPER)
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


class Time:
    def __init__(self, ns):
        self.ns = ns

    def to_nsec(self):
        return self.ns


def point(position, ns, velocity=.07, acceleration=-.03):
    return NS(positions=[position+i*.03 for i in range(6)],
              velocities=[velocity+i*.01 for i in range(6)],
              accelerations=[acceleration-i*.02 for i in range(6)],
              effort=[], time_from_start=Time(ns))


class Curve:
    def __init__(self):
        self.joint_trajectory = NS(
            header=NS(seq=17, stamp=Time(0), frame_id='base_link'),
            joint_names=list(m.ARM_NAMES),
            points=[point(-.2, 0), point(.1, 1123456789), point(.3, 2234567891)])
        self.multi_dof_joint_trajectory = NS(points=[])

    def serialize(self, stream):
        # An independent binary duck message, never a ROS connection.
        for p in self.joint_trajectory.points:
            stream.write(struct.pack('<Q4I', p.time_from_start.to_nsec(),
                len(p.positions), len(p.velocities), len(p.accelerations), len(p.effort)))
            values = p.positions+p.velocities+p.accelerations+p.effort
            stream.write(struct.pack('<%dd' % len(values), *values))


def inputs():
    model = '<robot name="frozen">\n  <link name="tool0"/>\n</robot>'
    digest = hashlib.sha256(model.encode()).hexdigest()
    scene = NS(plan_id='p-frozen', digest='a'*64, source_ns=10000000123,
               track_id='g7-t4', geometry_digest='b'*64,
               normal=[0., 0., 1.], offset=0., center=[.1, -.2, .02],
               rotation=np.eye(3), size=[.06, .045, .02])
    curve = Curve()
    hold = point(-.2, 0, velocity=0., acceleration=0.)
    hold.velocities = hold.accelerations = [0.]*6
    sample = dict(model_sha256=digest, joint_names=list(m.ARM_NAMES),
                  accepted_positions_rad=[-.2+i*.03 for i in range(6)],
                  sdk_positions_rad=[-.199+i*.03 for i in range(6)],
                  accepted_stamp_ns=10000100000, sdk_stamp_ns=10000090000,
                  accepted_gripper_opening_m=.0498)
    return dict(binding=dict(plan_id=scene.plan_id, plan_sha256=scene.digest,
                    source_stamp_ns=scene.source_ns, target_track_id=scene.track_id,
                    control_epoch_ns=9000000000,
                    selection=dict(mode='unknown', strategy='observe_return', generation=2),
                    stream_request_id=31, stream_generation=7, target_epoch=4,
                    near_field_phase_id=4, session_id='session-1', lease_id='lease-1'),
                checked_curve=curve, controller_desired=hold, scene=scene,
                model_xml=model, opening_m=.0498, opening_source='current_accepted_open_posture',
                following_sample=sample, config=dict(grasp=dict(enabled=True),
                    geometry=dict(support_clearance_m=.003), gripper=dict(open_skip_tolerance_m=.001)),
                first_failed_sample=dict(available=True, stage='grasp', curve='nominal',
                    label='8-end', joint_positions_rad=list(curve.joint_trajectory.points[1].positions),
                    T_base_tool0=np.eye(4).tolist(), body='right_finger',
                    displacement_bound_m=0., support_clearance_m=.005),
                failure=dict(stage='grasp', code='CONTACT_SEQUENCE_PRECHECK_FAILED',
                    message='nominal continuous geometry: target OBB collision at 8-end/right_finger'),
                plan_data=dict(poses_xyz_xyzw=[[.1, .2, .3, 0., 0., 0., 1.]]*4),
                other_curves={'predicted_command': deepcopy(curve)})


def payload():
    return m.contact_failure_data(**inputs())


def store(tmp_path):
    # Only the OS mount/UUID adapter is fake; the selected path and all file
    # content, canonicalization, gzip, readback and atomic publication are real.
    m.select_storage(path=tmp_path).validate(tmp_path)
    return m.DiagnosticStore(tmp_path/'objects')


@pytest.mark.parametrize('backend', ['ssd', 'sanstand'])
def test_each_explicit_backend_roundtrips_the_same_canonical_gzip(storage_os, backend):
    selected = storage_os.select(backend)
    sink = m.DiagnosticStore(selected.archive_root/'objects')
    original = payload(); reference = sink.put(original)
    assert sink.get(reference) == original
    assert sink._storage_binding['backend'] == backend
    packed = (sink.root/reference['relative_path']).read_bytes()
    assert gzip.decompress(packed) == m.canonical_bytes(original)
    other = storage_os.select('sanstand' if backend == 'ssd' else 'ssd')
    assert not other.archive_root.exists()


def test_default_store_selects_ssd_without_using_sanstand(storage_os):
    sink = m.DiagnosticStore(); reference = sink.put(payload())
    assert sink.root == storage_os.selected.archive_root/'replay_objects'
    assert sink.get(reference)['execution_authorized'] is False
    assert not storage_os.select('sanstand').archive_root.exists()


@pytest.mark.parametrize('fault', ['unmounted','remounted','wrong_uuid','config_changed'])
def test_frozen_store_rejects_disk_changes_and_preserves_published_bytes(storage_os, fault):
    sink = m.DiagnosticStore(storage_os.selected.archive_root/'objects')
    original = payload(); reference = sink.put(original)
    path = sink.root/reference['relative_path']; before = path.read_bytes()
    if fault == 'unmounted': storage_os.states['ssd']['mounted'] = False
    elif fault == 'remounted': storage_os.states['ssd']['mount_id'] += 1
    elif fault == 'wrong_uuid': storage_os.states['ssd']['uuid'] = 'foreign-volume'
    else: storage_os.config.write_text(storage_os.config.read_text()+' ')
    changed = deepcopy(original); changed['opening_m'] -= .0001
    with pytest.raises(m.ReplayStorageUnavailable): sink.put(changed)
    with pytest.raises(m.ReplayStorageUnavailable): sink.get(reference)
    assert path.read_bytes() == before and len(list(sink.root.rglob('*.json.gz'))) == 1
    assert not storage_os.select('sanstand').archive_root.exists()


def spline_kernel():
    path = ROOT/'src/alicia_flexible_grasp/robot/observation_path_guard.py'
    tree = ast.parse(path.read_text())
    ns = dict(np=np, math=math, ObservationPathError=ValueError)
    wanted = {'_array', 'segment_coefficients', '_polynomial'}
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)
         and n.name in wanted], type_ignores=[]), str(path), 'exec'), ns)
    return ns


def test_lossless_gzip_bytes_hash_and_actual_spline_replay(tmp_path):
    values = inputs()
    before = m.contact_failure_data(**values)
    s = store(tmp_path)
    ref = s.put(before)
    packed = (Path(ref['store_root'])/ref['relative_path']).read_bytes()
    raw = gzip.decompress(packed)
    assert hashlib.sha256(packed).hexdigest() == ref['gzip_sha256']
    assert hashlib.sha256(raw).hexdigest() == ref['object_sha256']
    assert raw == m.canonical_bytes(before)
    restored = s.get(ref)
    assert restored == json.loads(raw)
    assert restored['binding'] == before['binding']
    assert restored['scene'] == before['scene']
    assert restored['model'] == before['model']
    assert restored['config'] == before['config']
    assert restored['following_sample'] == before['following_sample']
    assert restored['first_failed_sample'] == before['first_failed_sample']
    assert restored['controller_desired']['time_from_start_ns'] == 0
    assert restored['checked_curve']['header'] == dict(seq=17, stamp_ns=0, frame_id='base_link')
    k = spline_kernel()
    original = values['checked_curve'].joint_trajectory.points
    recovered = [NS(**p) for p in restored['checked_curve']['points']]
    duration = restored['checked_curve']['points'][1]['time_from_start_ns']/1e9
    first = k['segment_coefficients'](original[0], original[1], duration, 6)
    second = k['segment_coefficients'](recovered[0], recovered[1], duration, 6)
    np.testing.assert_array_equal(first, second)
    for fraction in (0., .137, .5, 1.):
        np.testing.assert_array_equal(k['_polynomial'](first, fraction), k['_polynomial'](second, fraction))
    without_derivatives = deepcopy(recovered)
    for p in without_derivatives:
        p.velocities = p.accelerations = []
    linear = k['segment_coefficients'](*without_derivatives[:2], duration, 6)
    assert not np.allclose(k['_polynomial'](linear, .137), k['_polynomial'](second, .137))
    assert restored['execution_authorized'] is False and restored['diagnostic_only'] is True


def test_repeated_write_is_content_addressed_and_not_rewritten(tmp_path):
    s = store(tmp_path)
    data = payload()
    first = s.put(data)
    path = Path(first['store_root'])/first['relative_path']
    before = (path.stat().st_mtime_ns, path.read_bytes())
    assert s.put(deepcopy(data)) == first
    assert (path.stat().st_mtime_ns, path.read_bytes()) == before
    assert len(list(s.root.rglob('*.json.gz'))) == 1


def test_same_plain_content_with_different_gzip_encoding_is_kept(tmp_path):
    s = store(tmp_path); data = payload(); ref = s.put(data)
    path = Path(ref['store_root'])/ref['relative_path']
    alternate = gzip.compress(m.canonical_bytes(data), compresslevel=1, mtime=17)
    assert alternate != path.read_bytes()
    path.write_bytes(alternate)
    updated = s.put(data)
    assert updated['object_sha256'] == ref['object_sha256']
    assert updated['gzip_sha256'] == hashlib.sha256(alternate).hexdigest()
    assert path.read_bytes() == alternate and s.get(updated) == data


@pytest.mark.parametrize('change', ['source', 'curve', 'derivative', 'opening', 'model'])
def test_changed_frozen_inputs_create_distinct_object(tmp_path, change):
    s = store(tmp_path)
    one = payload(); two = deepcopy(one)
    if change == 'source':
        two['binding']['source_stamp_ns'] += 1
        two['scene']['source_ns'] += 1
    elif change == 'curve':
        two['checked_curve']['points'][1]['positions'][2] += .0001
    elif change == 'derivative':
        two['checked_curve']['points'][1]['accelerations'][0] += .001
    elif change == 'opening':
        two['opening_m'] -= .0001
    else:
        two['model']['xml'] += '\n'
        two['model']['sha256'] = hashlib.sha256(two['model']['xml'].encode()).hexdigest()
        two['following_sample']['model_sha256'] = two['model']['sha256']
    assert s.put(one)['object_sha256'] != s.put(two)['object_sha256']
    assert len(list(s.root.rglob('*.json.gz'))) == 2


def test_missing_mount_does_not_create_store_or_fallback(tmp_path, storage_os):
    s = store(tmp_path)
    storage_os.states['ssd']['mounted'] = False
    with pytest.raises(m.ReplayStorageUnavailable, match='no fallback'):
        s.put(payload())
    assert not s.root.exists()


def test_mount_loss_before_publish_has_no_committed_object(tmp_path, monkeypatch):
    s = store(tmp_path)
    original = s._ready
    calls = [0]
    def ready():
        calls[0] += 1
        if calls[0] >= 3:
            raise m.ReplayStorageUnavailable('simulated detach before publication')
        return original()
    monkeypatch.setattr(s, '_ready', ready)
    with pytest.raises(m.ReplayStorageUnavailable):
        s.put(payload())
    assert not list(s.root.rglob('*.json.gz'))
    assert not list(s.root.rglob('.*-*'))


def test_concurrent_duplicate_writers_publish_one_complete_object(tmp_path):
    s = store(tmp_path)
    data = payload()
    with ThreadPoolExecutor(max_workers=8) as pool:
        references = list(pool.map(lambda _: s.put(deepcopy(data)), range(24)))
    assert all(ref == references[0] for ref in references)
    assert s.get(references[0]) == data
    assert len(list(s.root.rglob('*.json.gz'))) == 1


@pytest.mark.parametrize('fault', ['nan', 'infinite_config', 'wrong_names', 'short_positions',
    'short_velocity', 'time_order', 'boolean_time', 'source_mismatch', 'wrong_model',
    'bad_first_fk', 'authority', 'boolean_schema', 'missing_failure'])
def test_malformed_payload_never_creates_a_directory(tmp_path, fault):
    data = payload()
    if fault == 'nan': data['checked_curve']['points'][0]['positions'][0] = float('nan')
    if fault == 'infinite_config': data['config']['new'] = float('inf')
    if fault == 'wrong_names': data['checked_curve']['joint_names'].reverse()
    if fault == 'short_positions': data['checked_curve']['points'][0]['positions'].pop()
    if fault == 'short_velocity': data['checked_curve']['points'][0]['velocities'].pop()
    if fault == 'time_order': data['checked_curve']['points'][1]['time_from_start_ns'] = 0
    if fault == 'boolean_time': data['checked_curve']['points'][1]['time_from_start_ns'] = True
    if fault == 'source_mismatch': data['binding']['source_stamp_ns'] += 1
    if fault == 'wrong_model': data['following_sample']['model_sha256'] = 'f'*64
    if fault == 'bad_first_fk': data['first_failed_sample']['T_base_tool0'].pop()
    if fault == 'authority': data['execution_authorized'] = True
    if fault == 'boolean_schema': data['schema_version'] = True
    if fault == 'missing_failure': data['failure'].pop('message')
    s = store(tmp_path)
    with pytest.raises(m.ReplayDataError):
        s.put(data)
    assert not s.root.exists()


def test_existing_corruption_and_tampered_reference_are_not_accepted(tmp_path):
    s = store(tmp_path); data = payload(); ref = s.put(data)
    bad = deepcopy(ref); bad['relative_path'] = '../escaped'
    with pytest.raises(m.ReplayDataError): s.get(bad)
    path = Path(ref['store_root'])/ref['relative_path']
    path.write_bytes(b'broken existing evidence')
    with pytest.raises(m.ReplayDataError): s.get(ref)
    with pytest.raises(m.ReplayDataError): s.put(data)
    assert path.read_bytes() == b'broken existing evidence'


def test_resolved_subdirectory_cannot_redirect_store(tmp_path, monkeypatch):
    s = store(tmp_path); s.root.mkdir()
    other = tmp_path/'other'; other.mkdir()
    # Exercise the resolver boundary for either filesystem without requiring
    # the fake SanStand mount to implement symlink creation.
    original = Path.resolve
    def redirected(path, *args, **kwargs):
        resolved = original(path, *args, **kwargs)
        try:
            suffix = resolved.relative_to(s.root/'sha256')
        except ValueError:
            return resolved
        return other/suffix
    monkeypatch.setattr(Path, 'resolve', redirected)
    with pytest.raises(m.ReplayStorageUnavailable): s.put(payload())
    assert not list(other.iterdir())


def test_empty_derivatives_and_missing_sample_are_explicit_not_invented():
    values = inputs()
    for p in values['checked_curve'].joint_trajectory.points:
        p.velocities = p.accelerations = []
    values['first_failed_sample'] = dict(available=False, reason='checker rejected before FK')
    data = m.contact_failure_data(**values)
    assert data['checked_curve']['points'][0]['velocities'] == []
    assert data['first_failed_sample'] == values['first_failed_sample']


def test_failing_sink_preserves_the_original_failure_object_and_code():
    class Original(ValueError): code = 'ORIGINAL_COLLISION'
    error = Original('original geometry rejection')
    bad_store = NS(put=lambda _: (_ for _ in ()).throw(OSError('disk unavailable')))
    with pytest.raises(Original) as caught:
        try:
            raise error
        except Original as exc:
            receipt = m.best_effort_failure_receipt(exc, payload, bad_store)
            raise
    assert caught.value is error and caught.value.code == 'ORIGINAL_COLLISION'
    assert receipt['status'] == 'unavailable'
    assert receipt['original_error_message'] == str(error)
    assert str(error) == 'original geometry rejection'


def test_baseexception_boundary_is_not_hidden_by_best_effort_sink():
    class Boundary(BaseException): pass
    def forbidden(): raise Boundary('offline external boundary')
    with pytest.raises(Boundary):
        m.best_effort_failure_receipt(ValueError('original'), forbidden, NS(put=lambda v: v))


def surface_inputs():
    identity = NS(epoch=4, track_id='g7-t4')
    surface = NS(identity=identity, points_base=np.array([[.01, .02, .03], [.02, .01, .04]]),
                 view_indices=[0, 1], view_stamps_ns=(10000000001, 10000000003),
                 view_sources=((0, 1), (1,)))
    reference = NS(identity=identity, stamp_ns=10000000001,
                   points_base=np.array([[.01, .02, .03], [.02, .02, .03], [.03, .02, .03]]),
                   support_normal_base=[0., 0., 1.], support_offset_m=.001,
                   edge_clearance_px=8)
    return surface, reference


def test_fused_surface_and_exact_reference_round_trip_without_union_substitution(tmp_path):
    surface, reference = surface_inputs()
    data = m.surface_data(surface, reference, snapshot_stamp_ns=10000000003,
        coordinate_frame='current_snapshot_base_estimate', snapshot_to_reference=np.eye(4),
        request_binding=dict(request_id=31, stream_generation=7, target_epoch=4))
    s = store(tmp_path); ref = s.put(data); recovered = s.get(ref)
    np.testing.assert_array_equal(recovered['points_base'], surface.points_base)
    np.testing.assert_array_equal(recovered['reference']['points_base'], reference.points_base)
    assert recovered['view_sources'] == [[0, 1], [1]]
    assert recovered['view_indices'] == [0, 1]
    assert recovered['view_stamps_ns'] == list(surface.view_stamps_ns)
    assert recovered['reference']['support_normal_base'] == reference.support_normal_base
    assert recovered['reference']['support_offset_m'] == reference.support_offset_m
    assert s.put(data) == ref


@pytest.mark.parametrize('fault', ['length', 'source_set', 'source_order', 'reference_identity'])
def test_malformed_fused_provenance_is_rejected_without_write(tmp_path, fault):
    surface, reference = surface_inputs()
    if fault == 'length': surface.view_indices = [0]
    if fault == 'source_set': surface.view_sources = ((0,), (0,))
    if fault == 'source_order': surface.view_stamps_ns = (10000000003, 10000000001)
    if fault == 'reference_identity': reference.identity = NS(epoch=5, track_id='g7-t5')
    s = store(tmp_path)
    with pytest.raises(m.ReplayDataError):
        data = m.surface_data(surface, reference, snapshot_stamp_ns=10000000003,
            coordinate_frame='current_snapshot_base_estimate', snapshot_to_reference=np.eye(4))
        s.put(data)
    assert not s.root.exists()


def test_helper_imports_no_ros_network_control_or_temperature_interfaces():
    tree = ast.parse(HELPER.read_text())
    modules = {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    modules.update(alias.name for n in ast.walk(tree) if isinstance(n, ast.Import) for alias in n.names)
    assert not modules.intersection({'rospy', 'rosbag', 'socket', 'serial', 'actionlib'})
    calls = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert not calls.intersection({'ServiceProxy', 'Publisher', 'send_goal', 'set_param'})
