"""Exact pure-value memo contracts retain every live ownership checkpoint."""
from copy import deepcopy
from dataclasses import fields
from unittest.mock import Mock
import numpy as np
import pytest
from test_observation_measurement_unknown import frozen, module


@pytest.mark.parametrize('before,after', [(1,1.),(1,True),(False,0),(0.,-0.),([1],(1,)),
    (np.array([1],np.int32),np.array([1],np.int64)),
    (np.array([0.]),np.array([-0.])),(np.array(1),np.array(2))])
def test_original_type_and_byte_changes_recompute(before,after):
    memo=module._ExactValueMemo();calls=[]
    def compute(value):calls.append(True);return len(calls)
    assert memo.get(before,compute)==1
    assert memo.get(deepcopy(before),compute)==1
    assert memo.get(after,compute)==2


def test_memo_owns_all_nested_values_and_rejects_mid_copy_change():
    value={'a':[np.array([1.,2.]), {'b':(1,2)}]};memo=module._ExactValueMemo()
    calls=[]
    def compute(snapshot):calls.append(True);return module.digest({'x':snapshot['a'][0].tolist()})
    old=memo.get(value,compute);value['a'][0][0]=3.
    assert memo.get(value,compute)!=old and len(calls)==2
    memo=module._ExactValueMemo()
    def mutate(snapshot):value['a'][1]['b']=(8,9);return 'old snapshot'
    with pytest.raises(ValueError,match='changed_during_snapshot'):memo.get(value,mutate)
    assert memo._entry is None


@pytest.mark.parametrize('value',[float('nan'),float('inf'),object()])
def test_unsupported_values_never_become_cache_acceptance(value):
    memo=module._ExactValueMemo();calls=[]
    def original_validator(snapshot):calls.append(True);raise ValueError('original invalid input')
    for _ in range(2):
        with pytest.raises(ValueError,match='original invalid input'):memo.get(value,original_validator)
    assert len(calls)==2 and memo._entry is None


def test_unchanged_binding_reuses_only_pure_computation(monkeypatch):
    n,clock,_,_,params,_=frozen(monkeypatch);pending=n._observation_measurement
    for _ in range(3):n._check_measurement_binding(pending)
    names=['_candidate_fingerprint','digest','camera_geometry_snapshot','_camera_configuration_contract']
    spies={name:Mock(wraps=getattr(module,name))for name in names}
    for name,spy in spies.items():monkeypatch.setattr(module,name,spy)
    tf=Mock(wraps=n.buffer.lookup_transform);monkeypatch.setattr(n.buffer,'lookup_transform',tf)
    get=Mock(wraps=module.rospy.get_param_cached);monkeypatch.setattr(module.rospy,'get_param_cached',get)
    for _ in range(7):n._check_measurement_binding(pending)
    assert all(spy.call_count==0 for spy in spies.values())
    assert tf.call_count==14
    for key in ['/gui/joint_direct_mode','/robot_description','/grasp_mode/selection','/camera','/unknown_perception','/handeye']:
        assert sum(call.args[0]==key for call in get.call_args_list)>=7


@pytest.mark.parametrize('field',[field.name for field in fields(module.Config)])
def test_every_config_field_still_checked_after_warming(monkeypatch,field):
    n,clock,_,_,_,_=frozen(monkeypatch);pending=n._observation_measurement
    n._check_measurement_binding(pending)
    setattr(n.config,field,getattr(n.config,field)+1)
    with pytest.raises(ValueError,match='identity_changed'):n._check_measurement_binding(pending)


def test_returned_context_tree_is_owned_and_preserves_tuple_types(monkeypatch):
    n,clock,_,_,params,_=frozen(monkeypatch)
    params['/unknown_perception']={'custom':('tuple',[1,{'inner':[2]}])}
    stamp=module.rospy.Time.now();a=n._rgb_interest_source_context(stamp)
    assert type(a['perception']['custom']) is tuple
    a['perception']['custom'][1][1]['inner'][0]=99
    b=n._rgb_interest_source_context(stamp)
    assert b['perception']['custom'][1][1]['inner']==[2]
    assert params['/unknown_perception']['custom'][1][1]['inner']==[2]


def test_allowed_telemetry_churn_recomputes_original_contract_without_false_rejection(monkeypatch):
    n,clock,_,_,params,_=frozen(monkeypatch,camera_profile=True);pending=n._observation_measurement
    n._check_measurement_binding(pending)
    original=module.rospy.get_param_cached;reads=[]
    def changing(name,*args):
        value=original(name,*args)
        if name=='/camera':
            reads.append(True)
            value['runtime_profile']['stationary_temporal']['active']=len(reads)%2==0
            value['runtime_profile']['stationary_temporal']['filtered_pixel_count']=len(reads)
        return value
    monkeypatch.setattr(module.rospy,'get_param_cached',changing)
    contract=Mock(wraps=module._camera_configuration_contract)
    monkeypatch.setattr(module,'_camera_configuration_contract',contract)
    for _ in range(7):n._check_measurement_binding(pending)
    assert len(reads)>=14 and contract.call_count==7
