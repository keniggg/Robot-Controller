"""Pure fake-message tests; do not import ROS or access any live devices."""
from pathlib import Path
import ast
import importlib.util
import json
from types import SimpleNamespace as N
import threading
import zipfile
import zlib

import pytest

PATH = Path(__file__).with_name('record_current_support_rgbd_20261007.py')
spec = importlib.util.spec_from_file_location('current_support_recorder_tested', PATH)
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


class Stamp:
    def __init__(self, ns): self.ns = ns
    def to_nsec(self): return self.ns


class Image:
    _type, _md5sum = 'sensor_msgs/Image', 'image-md5'
    def __init__(self, ns, stream='color', frame='camera_link'):
        self.header = N(stamp=Stamp(ns), seq=2, frame_id=frame)
        self.encoding = 'bgr8' if stream == 'color' else '16UC1'
        self.width, self.height = 4, 4
        self.step = self.width*(3 if stream == 'color' else 2)
        self.data = bytes(range(self.step*self.height))
    def serialize(self, handle): handle.write(self.data)


def add(cache, stamp, stream, now=1., frame='camera_link'):
    message = Image(stamp, stream, frame)
    cache.add(stream, message, stamp+1, now)
    return message


def joints(ns=100, value=0., frame='sdk_measured'):
    return N(header=N(stamp=Stamp(ns), seq=2, frame_id=frame),
             name=list(r.ARM), position=[value]*len(r.ARM))


def transform(ns=100, parent='base_link', child='camera_link'):
    return N(header=N(stamp=Stamp(ns), frame_id=parent), child_frame_id=child,
        transform=N(translation=N(x=0., y=0., z=0.), rotation=N(x=0., y=0., z=0., w=1.)))


def diagnostic(stamp=100):
    payload = dict(schema=1, capacity=32, geometry_generation=1,
        diagnostic_only=True, motion_authorized=False, entries=[dict(
            source_stamp_ns=stamp, geometry_generation=1, frame_id='camera_link', evidence={})])
    value = dict(schema=1, encoding='json', payload_json=json.dumps(payload))
    return dict(parameter='/camera_read_diagnostics', original_parameter_value=value,
                annotation=r.meta.parameter_annotation(value))


def test_exact_pair_never_uses_nearest_stamp():
    cache = r.base.ExactPairCache()
    add(cache, 100, 'color'); add(cache, 101, 'depth')
    assert r.choose_pair(cache, 1.3, 0, -10, 1.) is None
    add(cache, 100, 'depth')
    rows = r.choose_pair(cache, 1.3, 0, -10, 1.)
    assert [row['source_stamp_ns'] for row in rows] == [100, 100]


def test_minimum_receipt_settle_and_fixed_cadence():
    cache = r.base.ExactPairCache()
    add(cache, 100, 'color'); add(cache, 100, 'depth')
    assert r.choose_pair(cache, 1.1, 0, -10, 1.) is None
    assert r.choose_pair(cache, 1.3, 99, .5, 1.) is None
    assert r.choose_pair(cache, 1.3, 100, -10, 1.) is None
    assert r.choose_pair(cache, 1.3, 0, -10, 1.) is not None


def test_frame_geometry_mismatch_rejected():
    cache = r.base.ExactPairCache()
    add(cache, 100, 'color'); add(cache, 100, 'depth', frame='another_camera')
    assert r.choose_pair(cache, 1.3, 0, -10, 1.) is None


def test_old_pairs_expire():
    cache = r.base.ExactPairCache(seconds=1.5)
    add(cache, 100, 'color'); add(cache, 100, 'depth')
    assert r.choose_pair(cache, 3., 0, -10, 1.) is None


@pytest.mark.parametrize('field,value', [('encoding','rgb8'), ('width',0), ('step',1), ('data',b'x')])
def test_invalid_image_contract(field, value):
    image = Image(100)
    setattr(image, field, value)
    with pytest.raises(ValueError): r.image_contract(image, 'color')


def test_both_image_contracts_and_byte_ceiling():
    r.image_contract(Image(100), 'color')
    r.image_contract(Image(100, 'depth'), 'depth')
    image = Image(100)
    image.step, image.height = r.MAX_IMAGE+1, 1
    image.data = bytes(image.step)
    with pytest.raises(ValueError): r.image_contract(image, 'color')


def test_accepted_preserves_names_positions_no_temperature_fields():
    value = r.joint_row(joints(), dict(monotonic=1))
    assert value['names'] == list(r.ARM)
    assert set(value) == {'source_stamp_ns','seq','frame_id','names','positions','receipt','scope'}
    with pytest.raises(ValueError): r.joint_row(joints(frame='estimated'), {})


def test_exact_bracket_reports_delta_but_no_static_claim():
    rows = [r.joint_row(joints(90), {}), r.joint_row(joints(110, .2), {})]
    value = r.bracket(rows, 100, 1)
    assert value['source_bracket_available'] is True
    assert value['endpoint_position_delta']['Joint5'] == .2
    assert value['physical_stationarity_established'] is False
    assert value['interpolation_applied'] is False


@pytest.mark.parametrize('stamp,epoch', [(89,1), (111,1), (100,None), (100,91)])
def test_feedback_missing_or_epoch_incompatible_explicit(stamp, epoch):
    rows = [r.joint_row(joints(90), {}), r.joint_row(joints(110), {})]
    assert r.bracket(rows, stamp, epoch)['source_bracket_available'] is False


def test_feedback_span_not_widened():
    rows = [r.joint_row(joints(1), {}), r.joint_row(joints(300_000_001), {})]
    assert r.bracket(rows, 100, 1)['source_bracket_available'] is False


@pytest.mark.parametrize('ns,parent,child', [(101,'base_link','camera_link'),
    (100,'another_base','camera_link'), (100,'base_link','other_camera')])
def test_tf_wrong_stamp_or_frame_rejected(ns, parent, child):
    result = r.tf_audit(transform(ns,parent,child), 100,'base_link','camera_link')
    assert not result['exact_valid']
    assert result['returned_stamp_ns'] == ns
    assert result['latest_time_substitution'] is False


def test_exact_tf_and_bad_quaternion():
    assert r.tf_audit(transform(),100,'base_link','camera_link')['exact_valid']
    message = transform()
    message.transform.rotation.w = 2
    assert not r.tf_audit(message,100,'base_link','camera_link')['exact_valid']


def test_original_rpc_parameter_value_not_rewritten():
    value = diagnostic()['original_parameter_value']
    proxy = N(getParam=lambda caller, key: (1,'original message',value))
    record = r.read_parameter(proxy, '/recorder', '/camera_read_diagnostics', lambda:dict(ros_ns=50))
    assert record['original_parameter_value'] is value
    assert record['original_parameter_value']['payload_json'] == value['payload_json']
    assert [v['source_stamp_ns'] for v in r.diagnostic_rows(record)] == [100]


def test_invalid_envelope_does_not_index_arbitrary_json():
    record = diagnostic()
    record['annotation'] = {'schema_valid':False}
    assert r.diagnostic_rows(record) == []


def test_archive_pair_atomic_budget_and_serialization_roundtrip(tmp_path):
    compressed, metadata = r.base.serialize(Image(100))
    assert zlib.decompress(compressed) == Image(100).data
    assert metadata['serialized_sha256'] == r.base.sha(Image(100).data)
    archive = r.base.BoundedArchive(tmp_path/'small.zip', maximum_bytes=1200, status_reserve=0)
    assert not archive.put_many([('color',b'x'*500),('depth',b'x'*500)],category='images')
    archive.close()
    with zipfile.ZipFile(tmp_path/'small.zip') as check: assert not check.namelist()


def test_cache_memory_bound_and_duplicate_replacement():
    cache = r.base.ExactPairCache(maximum_bytes=90)
    add(cache,100,'color'); add(cache,100,'color'); add(cache,100,'depth')
    assert cache.bytes <= 90
    assert cache.counts['duplicate_source_messages'] == 1
    add(cache,101,'color')
    assert cache.bytes <= 90
    assert cache.counts['memory_bound_evictions'] > 0


def test_source_index_retains_conflicting_exact_sources(tmp_path):
    rec = r.Recorder(tmp_path,15.,10,N(),None,N())
    rec.archive = r.base.BoundedArchive(tmp_path/'diagnostic.zip')
    first, second = diagnostic(), diagnostic()
    changed = json.loads(second['original_parameter_value']['payload_json'])
    changed['entries'][0]['evidence'] = {'changed':True}
    second['original_parameter_value']['payload_json'] = json.dumps(changed)
    rec.inbox.put(first,1000); rec.inbox.put(second,1000)
    rec.save_metadata()
    assert len(rec.sources[100]) == 2
    rec.archive.close()


def test_no_mutation_selection_or_ros_import_at_module_scope():
    source = PATH.read_text()
    tree = ast.parse(source)
    calls = {node.func.attr for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)}
    assert not {'Publisher','ServiceProxy','set_param','setParam','deleteParam','signal_shutdown'} & calls
    assert '/grasp_mode/selection' not in source and '/grasp_mode/select' not in source
    assert '/perception/unknown/detector_status' not in source
    for node in tree.body:
        if isinstance(node, (ast.Import,ast.ImportFrom)):
            names = [alias.name for alias in node.names]
            module = getattr(node, 'module', '') or ''
            assert not any(name in ('rospy', 'tf2_ros') for name in names+[module])
    assert r.MAX_ARCHIVE < r.MAX_DIRECTORY <= 40*1024*1024
    assert set(r.PARAMETERS) == {'/camera_read_diagnostics','/camera','/handeye','/unknown_perception','/robot_description'}


def test_pair_saved_with_missing_tf_and_feedback_is_not_accepted(tmp_path):
    args = N(parent_frame='base_link', camera_frame='camera_link')
    ros = N(Time=lambda sec,nsec: Stamp(sec*10**9+nsec), Duration=lambda value:value)
    def missing(*args): raise LookupError('no exact TF')
    rec = r.Recorder(tmp_path,15,10,ros,N(lookup_transform=missing),args)
    rec.archive = r.base.BoundedArchive(tmp_path/'diagnostic.zip')
    add(rec.cache,100,'color'); add(rec.cache,100,'depth')
    pair = r.choose_pair(rec.cache,1.3,0,-10,1.)
    assert rec.save_pair(pair)
    rec.archive.close()
    with zipfile.ZipFile(tmp_path/'diagnostic.zip') as saved:
        row = json.loads(saved.read('pairs/00.json'))
        assert row['source_stamp_ns'] == 100
        assert row['target_identity'] is None and row['motion_authorized'] is False
        assert row['at_source']['tf'] is None
        assert row['at_source']['tf_error'] == 'LookupError: no exact TF'
        assert not row['at_source']['feedback']['source_bracket_available']
        assert zlib.decompress(saved.read('pairs/00_color.rosmsg.zlib')) == Image(100).data
    assert not rec.saved[0]['exact_tf_valid'] and not rec.saved[0]['feedback_bracket_available']


def test_finite_fake_ros_run_seals_without_images_or_rpc(tmp_path, monkeypatch):
    import sys
    from types import ModuleType
    sensor, standard, tfmodule = ModuleType('sensor_msgs.msg'), ModuleType('std_msgs.msg'), ModuleType('tf2_ros')
    sensor.Image, sensor.JointState, standard.Header = Image, object, object
    listeners = []
    class Listener:
        def __init__(self, buffer): self.closed=False; listeners.append(self)
        def unregister(self): self.closed=True
    tfmodule.TransformListener = Listener
    monkeypatch.setitem(sys.modules,'sensor_msgs.msg',sensor)
    monkeypatch.setitem(sys.modules,'std_msgs.msg',standard)
    monkeypatch.setitem(sys.modules,'tf2_ros',tfmodule)
    subscribers = []
    def subscribe(topic, datatype, callback, **kwargs):
        obj = N(topic=topic, closed=False)
        obj.unregister=lambda:setattr(obj,'closed',True)
        subscribers.append(obj)
        return obj
    args = N(parent_frame='base_link', camera_frame='camera_link', color_topic='/color',
             depth_topic='/depth', confirmation_id='test_static_receipt_not_authority')
    ros = N(Subscriber=subscribe, is_shutdown=lambda:False)
    rec = r.Recorder(tmp_path,.03,5,ros,object(),args)
    rec.poll = lambda:None
    assert rec.run() == 2
    summary = json.loads((tmp_path/'summary.json').read_text())
    assert summary['sealed'] and summary['ready'] and summary['pair_count'] == 0
    assert summary['target_or_support_accepted'] is False
    assert summary['measurement_quality_not_evaluated'] is True
    assert all(sub.closed for sub in subscribers) and listeners[0].closed
    assert {sub.topic for sub in subscribers} == {'/color','/depth',
        '/alicia_d/accepted_joint_states','/alicia_d/control_reference_epoch'}
    assert summary['rpc_thread_alive'] is False
    assert (tmp_path/'SHA256SUMS').exists()
