import copy
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from alicia_grasp_modes.loss_observation import first_loss_observation


def fields():
    return dict(reason='ambiguous_locked_instances',generation=2,selection_stamp_ns=100,
                observed_stamp_ns=300,source_stamp_ns=250,anchor_source_stamp_ns=150,
                anchor_position_base=[.1,.2,.3],anchor_extent_m=[.03,.02,.01],
                reference_support_plane_base=[0.,0.,1.,-.1],last_good_source_stamp_ns=200,
                last_consumed_source_stamp_ns=250)


def test_original_reason_and_source_survive_generic_lost_status():
    first=first_loss_observation(None,**fields())
    later=fields();later.update(reason='target_lost_requires_new_generation',source_stamp_ns=500)
    assert first_loss_observation(first,**later)==first
    assert first['reason']=='ambiguous_locked_instances' and first['source_stamp_ns']==250


def test_stream_timeout_does_not_invent_image_source():
    data=fields();data.update(reason='exact_rgbd_stream_stale',source_stamp_ns=None)
    result=first_loss_observation(None,**data)
    assert result['source_stamp_ns'] is None and result['source_pair_available'] is False
    assert result['observed_stamp_ns']==300 and result['last_good_source_stamp_ns']==200


def test_snapshot_is_independent_of_mutable_anchor_and_input_record():
    data=fields();result=first_loss_observation(None,**data);frozen=copy.deepcopy(result)
    data['anchor_position_base'][0]=99
    later=first_loss_observation(result,**fields());later['anchor_position_base'][1]=99
    assert result==frozen


def test_only_explicit_new_context_can_make_a_new_first_record():
    old=first_loss_observation(None,**fields());new=fields();new.update(generation=3,source_stamp_ns=900)
    assert first_loss_observation(old,**new)==old
    assert first_loss_observation(None,**new)['generation']==3


def test_diagnostic_never_claims_recovery_or_mutates_external_tracker():
    result=first_loss_observation(None,**fields())
    assert result['diagnostic_only'] and result['target_recovered'] is False


def test_candidate_diagnostics_distinguish_zero_candidates_from_failed_identity():
    import numpy as np
    from types import SimpleNamespace as N
    from alicia_grasp_modes.loss_observation import candidate_loss_observation
    from alicia_grasp_modes.tabletop import Config, _matching_candidates
    anchor=N(position_base=np.array([0.,0.,0.]),extent_m=np.array([.02,.01,.01]))
    rejected=N(position_base=np.array([.026,0.,0.]),extent_m=np.array([.02,.01,.01]),quality=.9)
    valid=N(position_base=np.array([.01,0.,0.]),extent_m=np.array([.02,.01,.01]),quality=.9)
    config=Config()
    for candidates in ([],[rejected],[valid],[valid,valid]):
        result=N(candidates=candidates,metrics={'support_source':'locked_base_plane'})
        evidence=candidate_loss_observation(result,anchor,anchor,config,False)
        assert evidence['eligible_count']==len(_matching_candidates(candidates,anchor,anchor,config))
        assert evidence['candidate_count']==len(candidates)
        assert evidence['tracker_lost_before_choose'] is False
    assert candidate_loss_observation(N(candidates=[rejected],metrics={}),anchor,anchor,config,False)['candidates'][0]['anchor_distance_m']==.026
