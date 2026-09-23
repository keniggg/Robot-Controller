from copy import deepcopy
import types

import pytest
from alicia_grasp_modes.target_mass import bind_mass_evidence


def case():
    selected = dict(mode='unknown',strategy='direct',generation=6,stamp_ns=1_000_000_000)
    evidence = dict(selection=selected,anchor_source_stamp_ns='1100000000',reported_at_ns='1200000000',
        expires_at_ns='601200000000',estimated_mass_kg=.01,source='operator_estimate',operator_statement='重量为10g左右')
    status = dict(generation=6,anchor_source_stamp_ns=1_100_000_000,source_stamp_ns=1_800_000_000,
        state='ready',target_locked=True,target_lost=False)
    plan = types.SimpleNamespace(model_choice='unknown_tabletop',plan_id='plan-A',target_track_id='g4-t18',
        header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_nsec=lambda:1_600_000_000)))
    return {},plan,selected,evidence,status,2_000_000_000


def test_mass_evidence_is_copied_and_bound_to_exact_plan_and_target():
    args=case();before=deepcopy(args[3]);result=bind_mass_evidence(*args)
    assert args[0] == {} and args[3] == before
    bound=result['target_mass_evidence']
    assert bound['plan_id']=='plan-A' and bound['target_track_id']=='g4-t18'
    assert bound['estimated_mass_kg']==.01 and len(bound['evidence_sha256'])==64
    assert bound['snapshot_stamp_ns']=='1600000000'


@pytest.mark.parametrize('field,value', [('generation',7),('anchor_source_stamp_ns',1_100_000_001),
    ('target_locked',False),('target_lost',True),('state','unavailable'),('source_stamp_ns',999_999_999)])
def test_identity_loss_or_old_perception_cannot_authorize_mass(field,value):
    args=case();args[4][field]=value
    with pytest.raises(ValueError,match='identity'):
        bind_mass_evidence(*args)


@pytest.mark.parametrize('field,value', [('estimated_mass_kg',0),('estimated_mass_kg',float('nan')),
    ('estimated_mass_kg',True),('source','guessed_by_colour'),('operator_statement',''),
    ('expires_at_ns','1999999999'),('reported_at_ns','2000000001')])
def test_invalid_or_expired_operator_evidence_rejected(field,value):
    args=case();args[3][field]=value
    with pytest.raises(ValueError):
        bind_mass_evidence(*args)


def test_new_selection_and_carton_cannot_inherit_old_mass():
    args=list(case());args[2]=dict(args[2],generation=7)
    assert bind_mass_evidence(*args) is args[0]
    args[2]=dict(args[2],mode='carton')
    assert bind_mass_evidence(*args) is args[0]


def test_planning_latency_does_not_expire_identity_mass_before_plan_authority():
    args=list(case())
    args[5]=60_000_000_000
    # The same anchor was observed after the exact frozen snapshot. Plan-age
    # admission remains the caller's existing gate, not a second mass gate.
    assert bind_mass_evidence(*args)['target_mass_evidence']['estimated_mass_kg']==.01


def test_observation_before_plan_snapshot_cannot_bind_mass():
    args=case();args[4]['source_stamp_ns']=1_599_999_999
    with pytest.raises(ValueError,match='identity'):
        bind_mass_evidence(*args)
