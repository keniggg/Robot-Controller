"""Real Noetic child-cache behavior cannot truncate planner configuration."""
import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import pytest
import rospy.client as client
import rospy.impl.paramserver as paramserver
from rospy.msproxy import MasterProxy

from alicia_flexible_grasp.utils.authoritative_parameters import read_authoritative_mapping
from test_remote_grasp6d_node import remote_node as remote, DummyLock
from test_runtime_refresh_callback_transaction import runtime_node


LEAF = '/grasp/unknown_require_measured_bilateral_contact'


class OfflineMaster:
    def __init__(self, parameters):
        self.parameters = deepcopy(parameters)
        self.calls = []
        self.failure = None

    def value(self, key):
        if key in self.parameters:
            return deepcopy(self.parameters[key])
        parent, name = key.rsplit('/', 1)
        return deepcopy(self.parameters[parent][name])

    def getParam(self, caller, key):
        self.calls.append(('getParam', key))
        if self.failure is not None and key == '/grasp':
            if isinstance(self.failure, Exception):
                raise self.failure
            return deepcopy(self.failure)
        try:
            return 1, 'offline master', self.value(key)
        except KeyError:
            return -1, 'parameter missing', None

    def subscribeParam(self, caller, uri, key):
        self.calls.append(('subscribeParam', key))
        try:
            return 1, 'offline subscription', self.value(key)
        except KeyError:
            return -1, 'parameter missing', None


@pytest.fixture
def master(monkeypatch):
    full = dict(contact_height_registration_margin_enabled=True,
        handeye_translation_max_error_m=.0026767676132463574,
        unknown_require_measured_bilateral_contact=False,
        min_object_confidence=.9, max_object_jump_m=.025,
        object_jump_filter_window_sec=4.,
        nested={'preserve': [1, 2, 3]}, tool_approach_axis='z')
    server = OfflineMaster({'/grasp': full, '/grasp_6d/remote': {'request_hz': 2.}})
    proxy = MasterProxy.__new__(MasterProxy)  # no XML-RPC construction
    proxy.target = server
    cache = paramserver.ParamServerCache()
    monkeypatch.setattr(client, '_param_server', proxy)
    monkeypatch.setattr(paramserver, '_param_server_cache', cache)
    monkeypatch.setattr(remote.rospy, 'get_master', lambda: proxy)
    return server, proxy, cache, full


def test_child_cache_reproduces_old_failure_but_authoritative_reads_stay_complete(master):
    server, proxy, cache, full = master
    assert remote.rospy.get_param('/grasp') == full
    assert remote.rospy.get_param_cached(LEAF) is False
    partial = remote.rospy.get_param('/grasp')
    assert partial == {'unknown_require_measured_bilateral_contact': False}
    assert remote.RemoteGrasp6DNode._contact_height_evidence_limit(
        SimpleNamespace(grasp_config=partial)) is None
    calls = len(server.calls)
    for _ in range(2):
        config = remote.RemoteGrasp6DNode._read_grasp_config()
        assert config == full
        assert remote.RemoteGrasp6DNode._contact_height_evidence_limit(
            SimpleNamespace(grasp_config=config)) == full['handeye_translation_max_error_m']
    assert server.calls[calls:] == [('getParam', '/grasp')]*2
    assert cache.get('/grasp') == partial  # no clearing or repair of subscriptions
    assert remote.rospy.get_param_cached('/grasp') == partial


def test_complete_snapshot_is_independent_and_reads_current_master_updates(master):
    server, proxy, _, full = master
    first = read_authoritative_mapping(proxy, '/grasp')
    first['nested']['preserve'].append(4)
    assert server.parameters['/grasp'] == full
    server.parameters['/grasp']['nested']['preserve'].append(5)
    server.parameters['/grasp']['handeye_translation_max_error_m'] = .003
    second = read_authoritative_mapping(proxy, '/grasp')
    assert first['nested']['preserve'] == [1, 2, 3, 4]
    assert second['nested']['preserve'] == [1, 2, 3, 5]
    assert second['handeye_translation_max_error_m'] == .003


@pytest.mark.parametrize('failure', [(-1,'missing',None),(0,'error',{}),
    (True,'not integer',{}),(1,'bad mapping',None),(1,'bad mapping',[]),
    (1,'bad mapping','{}'),(1,'short'),RuntimeError('offline transport failure')])
def test_authoritative_failure_does_not_reuse_old_config_or_commit_stage(master, failure):
    server, _, _, _ = master
    node = runtime_node()
    original = deepcopy(node.grasp_config)
    rate = node.rate_hz
    server.failure = failure
    try:
        with pytest.raises(remote.CandidateContractError) as caught:
            node._refresh_runtime_params()
        assert caught.value.code == 'GRASP_CONFIG_UNAVAILABLE'
        assert node.grasp_config == original and node.rate_hz == rate
    finally:
        node.shutdown_streaming_worker()


def test_real_runtime_stage_keeps_height_margin_after_child_cache(master):
    _, _, _, full = master
    node = runtime_node()
    try:
        assert remote.rospy.get_param_cached(LEAF) is False
        node._refresh_runtime_params()
        assert node.grasp_config == full
        assert node._contact_height_evidence_limit() == full['handeye_translation_max_error_m']
        assert node.rate_hz == 2.
    finally:
        node.shutdown_streaming_worker()


def test_detection_uses_authoritative_nondefault_confidence_and_jump_after_child_cache(master, monkeypatch):
    _, _, _, _ = master
    remote.rospy.get_param_cached(LEAF)
    node = remote.RemoteGrasp6DNode.__new__(remote.RemoteGrasp6DNode)
    node._object_lock = DummyLock()
    node.latest_object = node.latest_object_time = None
    monkeypatch.setattr(remote.rospy.Time, 'now', staticmethod(lambda: remote.rospy.Time.from_sec(10.)))
    monkeypatch.setattr(remote.rospy, 'logwarn_throttle', lambda *a, **k: None)
    def obj(x, confidence):
        pose = remote.PoseStamped(); pose.pose.position.x=x; pose.pose.orientation.w=1.
        return SimpleNamespace(detected=True, confidence=confidence, pose_base=pose)
    node.object_cb(obj(.1, .75))
    assert node.latest_object is None  # old partial config defaulted to .5
    node.object_cb(obj(.1, .95))
    original = node.latest_object
    node.object_cb(obj(.14, .95))
    assert node.latest_object is original  # old partial config allowed .12 m


def test_startup_rejects_unavailable_namespace_before_live_surfaces(master, monkeypatch):
    server, _, _, _ = master
    server.failure=(-1,'missing',None)
    monkeypatch.setattr(remote, 'CvBridge', lambda: pytest.fail('no live node surfaces'))
    with pytest.raises(remote.CandidateContractError) as caught:
        remote.RemoteGrasp6DNode()
    assert caught.value.code == 'GRASP_CONFIG_UNAVAILABLE'


def test_all_four_parent_reads_use_authoritative_entry():
    tree = ast.parse(Path(remote.__file__).read_text())
    node = next(x for x in tree.body if isinstance(x, ast.ClassDef) and x.name=='RemoteGrasp6DNode')
    calls = []
    for method in node.body:
        if not isinstance(method, ast.FunctionDef):
            continue
        for call in ast.walk(method):
            if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute):
                if call.func.attr == '_read_grasp_config':
                    calls.append(method.name)
                if call.func.attr in ('get_param','get_param_cached') and call.args:
                    assert not (isinstance(call.args[0],ast.Constant) and call.args[0].value=='/grasp')
    assert sorted(calls) == ['__init__','__init__','_stage_runtime_params','object_cb']


@pytest.mark.parametrize('value', [False, True])
def test_live_leaf_updates_preserve_boolean_policy(master, value):
    server, _, cache, _ = master
    assert remote.RemoteGrasp6DNode._box_requires_measured_bilateral_contact() is False
    server.parameters['/grasp']['unknown_require_measured_bilateral_contact']=value
    cache.update(LEAF, value)  # actual rospy paramUpdate cache semantics
    assert remote.RemoteGrasp6DNode._box_requires_measured_bilateral_contact() is value


@pytest.mark.parametrize('value', [None,0,1,'false',{},[]])
def test_nonboolean_cached_leaf_never_enables_inference(master, value):
    server, _, cache, _ = master
    server.parameters['/grasp']['unknown_require_measured_bilateral_contact']=value
    cache.set(LEAF,value)
    with pytest.raises(remote.CandidateContractError) as caught:
        remote.RemoteGrasp6DNode._box_requires_measured_bilateral_contact()
    assert caught.value.code=='GRASP_CONFIG_UNAVAILABLE'


def test_missing_leaf_never_defaults_false(master):
    server, _, _, _=master
    del server.parameters['/grasp']['unknown_require_measured_bilateral_contact']
    with pytest.raises(remote.CandidateContractError) as caught:
        remote.RemoteGrasp6DNode._box_requires_measured_bilateral_contact()
    assert caught.value.code=='GRASP_CONFIG_UNAVAILABLE'


def test_leaf_transport_failure_never_defaults_false(master, monkeypatch):
    server, _, _, _=master
    def unavailable(*args):
        raise RuntimeError('offline leaf transport failure')
    monkeypatch.setattr(server,'subscribeParam',unavailable)
    with pytest.raises(remote.CandidateContractError) as caught:
        remote.RemoteGrasp6DNode._box_requires_measured_bilateral_contact()
    assert caught.value.code=='GRASP_CONFIG_UNAVAILABLE'


@pytest.mark.parametrize('kind', ['qualified_model', 'precision_candidate'])
@pytest.mark.parametrize('policy', ['false', 'true', 'nonboolean', 'missing', 'transport'])
def test_all_active_box_entries_use_strict_live_policy(monkeypatch, kind, policy):
    """Real qualified and unqualified sources cannot bypass a failed leaf read."""
    from dataclasses import replace
    from test_gen26_inferred_box_remote_pipeline import pipeline, remote as pipeline_remote
    from test_gen31_outer_rgb_box import precision_interior_scene, precision_scene_snapshot

    node, scene, prepared, _, _ = pipeline(monkeypatch)
    snapshot = prepared.snapshot
    geometry = scene.geometry
    if kind == 'precision_candidate':
        scene = precision_interior_scene()
        scene.K.depth_scale = .0001
        geometry = scene.geometry
        snapshot = precision_scene_snapshot(scene)
        node._latest_target_observation = replace(node._latest_target_observation,
            points_base=geometry.object_points_base)
        node._camera_intrinsics = lambda: scene.K
        node._prepare_regular_box_snapshot(snapshot, geometry, scene.T)
        accessor = lambda: node._current_box_precision_candidate(snapshot, geometry, scene.T)
    else:
        accessor = node._current_inferred_box
    assert accessor() is not None  # A genuine source exercises the policy branch.

    calls = []
    def read_policy(name):
        calls.append(name)
        assert name == LEAF
        if policy == 'missing':
            raise KeyError(name)
        if policy == 'transport':
            raise RuntimeError('offline subscription unavailable')
        return {'false': False, 'true': True, 'nonboolean': 0}[policy]
    monkeypatch.setattr(pipeline_remote.rospy, 'get_param_cached', read_policy)
    if policy in ('nonboolean', 'missing', 'transport'):
        with pytest.raises(pipeline_remote.CandidateContractError) as caught:
            accessor()
        assert caught.value.code == 'GRASP_CONFIG_UNAVAILABLE'
        with pytest.raises(pipeline_remote.CandidateContractError) as caught:
            node._prepare_regular_box_snapshot(snapshot, geometry, scene.T)
        assert caught.value.code == 'GRASP_CONFIG_UNAVAILABLE'
        assert node._regular_box_snapshot is None
    else:
        assert (accessor() is None) is (policy == 'true')
        node._prepare_regular_box_snapshot(snapshot, geometry, scene.T)
        assert (node._regular_box_snapshot is None) is (policy == 'true')
    assert calls == [LEAF, LEAF]
