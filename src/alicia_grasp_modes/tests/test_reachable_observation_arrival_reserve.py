"""Production view selection must reserve configured arrival uncertainty."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from alicia_grasp_modes import reachable_observation_views as module
from alicia_flexible_grasp.robot.observation_arrival_reserve import (
    observation_goal_error_bounds,arrival_reserve_from_source)
from test_reachable_observation_seed_replay import load_recorded_scene
from test_remote_modes import configured,base
from test_observe_return_remote import setup_observe_return
from alicia_grasp_modes.silhouette_observation_planning import SilhouetteObservationPlanningMixin


@pytest.fixture(scope='module')
def reserved():
    results={}
    for stem in ('supplement_view_joint_reserve_20261023','supplement_view_feasible_gain_20261023','supplement_view_seed_20261023'):
        args=load_recorded_scene(stem)
        args['endpoint_error_bounds_rad']=observation_goal_error_bounds(
            {n:dict(goal=.035) for n in args['fk'].names},args['fk'].names)
        args['max_seconds']=3.
        # This regression checks endpoint physics, independently of concurrent
        # worker wall time. The unchanged live budget has separate timed replays.
        args['clock']=lambda:0.
        rows,report=module.generate_reachable_observation_views(**args)
        assert rows,report
        results[stem]=(args,rows,report)
    return results


@pytest.mark.parametrize('stem',['supplement_view_joint_reserve_20261023',
    'supplement_view_feasible_gain_20261023','supplement_view_seed_20261023'])
def test_recorded_views_cover_arrival_error_without_losing_original_gates(reserved,stem):
    args,rows,report=reserved[stem]
    assert report['arrival_error_box_required']
    assert any(r['endpoint_checks']['missing_side_view_gain']['predicted_new_side_gain_m']>args['depth_uncertainty_m'] for r in rows)
    for row in rows:
        checks=row['endpoint_checks']
        for name in ('arrival_error_box_visibility','working_volume','observation_envelope',
                'conditional_endpoint_support','measured_support_projection','optical_side_evidence',
                'arrival_projection_endpoint','endpoint_joint_reserve','missing_side_view_gain'):
            assert checks[name]['ok'],(name,checks[name])
        assert checks['arrival_error_box_visibility']['nominal_minimum_image_edge_distance_px']>90
        assert row['execution_authority'] is False and row['complete_path_checked'] is False


def test_nominal_solver_success_cannot_override_arrival_reserve(monkeypatch):
    args=load_recorded_scene('supplement_view_joint_reserve_20261023')
    old,_=module.generate_reachable_observation_views(**args);assert old
    q=np.array(old[0]['q'])
    monkeypatch.setattr(module,'_backend',lambda:lambda *a,**k:NS(x=q,nit=1,success=True,message='untrusted success'))
    args['endpoint_error_bounds_rad']=np.full(6,.035+np.pi/4096)
    rows,report=module.generate_reachable_observation_views(**args)
    assert not rows
    checks=[r['endpoint_checks']['arrival_error_box_visibility'] for r in report['directions'] if 'endpoint_checks' in r]
    assert checks and all(not c['ok'] for c in checks)


def strict_context(configured,monkeypatch,reserved):
    node,params,_=setup_observe_return(configured)
    args,rows,_=reserved['supplement_view_joint_reserve_20261023'];row=rows[0]
    node._observe_return_phase='observation_prefix'
    node._observe_return_diagnostic_observation_only=False
    source=deepcopy(args['source']);prepared=object()
    monkeypatch.setattr(node,'_require_diagnostic_observation_view_source',lambda p:source)
    monkeypatch.setattr(node,'_frozen_observation_translation_delta_m',lambda *a:.05)
    model=args['fk'];joint=deepcopy(row['endpoint_checks']['endpoint_joint_reserve'])
    from pathlib import Path
    root=Path(__file__).resolve().parents[3]
    params['/robot_description']=(root/'deployment/2026-10-08/runtime/candidate_models/expanded_metric_20260929/alicia_expanded_metric_candidate.urdf').read_text()
    params['/alicia_controller/constraints']={n:dict(goal=.035,trajectory=.2) for n in model.names}
    report=dict(ok=True,endpoint_joint_reserve=joint,controller_constraints_snapshot=deepcopy(params['/alicia_controller/constraints']))
    result=base.MoveItResult(True,.4,1.,'strict checked')
    # The earlier continuous visibility/support qualification has separate
    # tests. Feed its exact branch evidence into the final selector here.
    monkeypatch.setattr(SilhouetteObservationPlanningMixin,'_observation_following_support_evaluation',lambda *a,**k:(result,deepcopy(report)))
    return node,params,source,prepared,result,report


def test_final_strict_branch_rechecks_configured_goal_error_box(configured,monkeypatch,reserved):
    node,params,source,prepared,result,report=strict_context(configured,monkeypatch,reserved)
    result,audit=node._observation_following_support_evaluation(prepared,result,object())
    assert result.reachable and audit['arrival_error_box_visibility']['ok']
    assert audit['controller_constraints_snapshot']==params['/alicia_controller/constraints']


@pytest.mark.parametrize('fault',['joint','constraints','model','source_changed','config_changed','old_edge','rgb_hole'])
def test_final_selection_fails_closed_on_missing_changed_or_fragile_bound(configured,monkeypatch,reserved,fault):
    node,params,source,prepared,result,report=strict_context(configured,monkeypatch,reserved)
    if fault=='joint':del report['endpoint_joint_reserve']
    elif fault=='constraints':del report['controller_constraints_snapshot']['Joint3']['goal']
    elif fault=='model':report['endpoint_joint_reserve']['model_sha256']='0'*64
    elif fault=='source_changed':
        counter=[0]
        def changing(p):
            counter[0]+=1
            return source if counter[0]==1 else deepcopy(source)
        monkeypatch.setattr(node,'_require_diagnostic_observation_view_source',changing)
    elif fault=='config_changed':params['/alicia_controller/constraints']['Joint2']['goal']=.04
    elif fault=='old_edge':
        args=load_recorded_scene('supplement_view_joint_reserve_20261023');old,_=module.generate_reachable_observation_views(**args)
        report['endpoint_joint_reserve']['goal_joint_positions_rad']=old[0]['q']
    else:source['arrival_projection']['rgb_valid'][:]=False
    result,audit=node._observation_following_support_evaluation(prepared,result,object())
    assert not result.reachable and not audit['ok']
    assert result.failure_code=='OBSERVATION_ENDPOINT_VIEW_RESERVE_INSUFFICIENT'
