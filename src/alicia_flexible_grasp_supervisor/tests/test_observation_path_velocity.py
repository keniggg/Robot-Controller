"""Offline ros_control bridge speed proofs; no ROS node or service calls."""
from copy import deepcopy

import numpy as np
import pytest

from alicia_flexible_grasp.robot.observation_path_guard import ObservationPathError
from test_observation_path_guard import point, trajectory


def validate(plan, desired, limits, **kwargs):
    from alicia_flexible_grasp.robot.observation_path_guard import (
        validate_observation_trajectory_velocity,
    )
    return validate_observation_trajectory_velocity(plan, desired, limits, **kwargs)


@pytest.mark.parametrize('degree,peak', [(1,.02),(3,.03),(5,.0375)])
def test_linear_cubic_quintic_speed_bound_covers_interior_peak(degree,peak):
    v=[0.] if degree>=3 else None
    a=[0.] if degree==5 else None
    plan=trajectory([point([0.],0.,v,a),point([.02],1.,v,a)])
    report=validate(plan,point([0.],v=[0.],a=[0.]),{'J1':.08})
    assert peak<=report['maximum_certified_speed_bound_rad_s']['J1']<=.08
    assert report['segments']==1 and report['trajectory_sha256']


@pytest.mark.parametrize('degree', [1,3,5])
def test_real_bridge_from_desired_is_checked_not_dropped_actual_t0(degree):
    v=[0.] if degree>=3 else None
    a=[0.] if degree==5 else None
    plan=trajectory([point([0.],0.,v,a),point([.001],.1,v,a)])
    # The trajectory's own t0-to-first rate is legal, the real bridge is not.
    validate(plan,point([0.],v=[0.],a=[0.]),{'J1':.08})
    with pytest.raises(ObservationPathError,match='bridge.*J1'):
        validate(plan,point([.06],v=[0.],a=[0.]),{'J1':.08})


@pytest.mark.parametrize('degree', [3,5])
def test_zero_waypoint_velocity_does_not_hide_later_segment_peak(degree):
    a=[0.] if degree==5 else None
    plan=trajectory([point([0.],0.,[0.],a),point([0.],1.,[0.],a),
                     point([.08],2.,[0.],a)])
    with pytest.raises(ObservationPathError,match='segment 1.*J1'):
        validate(plan,point([0.],v=[0.],a=[0.]),{'J1':.08})


def test_simultaneous_axes_keep_distinct_limits_and_joint_name_order():
    plan=trajectory([point([0.,0.],0.,[0.,0.],[0.,0.]),
                     point([.01,.03],1.,[0.,0.],[0.,0.])],['Joint6','Joint2'])
    desired=point([0.,0.],v=[0.,0.],a=[0.,0.])
    report=validate(plan,desired,{'Joint2':.08,'Joint6':.02})
    assert report['maximum_certified_speed_bound_rad_s']['Joint6']<=.02
    assert report['maximum_certified_speed_bound_rad_s']['Joint2']<=.08
    plan.joint_trajectory.points[1].positions[0]=.02
    with pytest.raises(ObservationPathError,match='Joint6'):
        validate(plan,desired,{'Joint2':.08,'Joint6':.02})


def test_positive_time_first_waypoint_is_the_bridge_endpoint():
    plan=trajectory([point([.01],1.),point([.02],2.)])
    report=validate(plan,point([0.],v=[0.],a=[0.]),{'J1':.02})
    assert report['segments']==2


@pytest.mark.parametrize('limits', [{},{'wrong':.08},{'J1':0.},
                                   {'J1':float('nan')},{'J1':True},
                                   {'J1':.08,'extra':.08}])
def test_incomplete_or_invalid_velocity_contract_is_rejected(limits):
    plan=trajectory([point([0.],0.),point([.01],1.)])
    with pytest.raises(ObservationPathError):
        validate(plan,point([0.],v=[0.],a=[0.]),limits)


@pytest.mark.parametrize('failure', ['budget','depth','time','moving','missing_acc','nan'])
def test_unresolved_or_invalid_speed_evidence_fails_closed(failure):
    plan=trajectory([point([0.],0.,[0.],[0.]),point([.02],1.,[0.],[0.])])
    desired=point([0.],v=[0.],a=[0.]);kwargs={}
    if failure=='budget':kwargs['max_checks']=0
    if failure=='depth':kwargs['max_depth']=0
    if failure=='time':kwargs.update(max_seconds=.001,clock=iter([0.,1.]).__next__)
    if failure=='moving':desired.velocities=[.001]
    if failure=='missing_acc':desired.accelerations=[]
    if failure=='nan':plan.joint_trajectory.points[-1].positions=[float('nan')]
    with pytest.raises(ObservationPathError):validate(plan,desired,{'J1':.04},**kwargs)


def test_velocity_proof_does_not_mutate_plan_or_desired():
    plan=trajectory([point([0.],0.,[0.],[0.]),point([.02],1.,[0.],[0.])])
    desired=point([0.],v=[0.],a=[0.]);before=deepcopy((plan,desired))
    validate(plan,desired,{'J1':.08})
    assert plan==before[0] and desired==before[1]


@pytest.mark.parametrize('degree', [1,3,5])
def test_structured_scale_and_existing_uniform_stretch_keep_bridge_geometry(degree):
    from alicia_flexible_grasp.robot.observation_path_guard import (
        ObservationVelocityLimitError, segment_coefficients, wire_digest,
    )
    from alicia_flexible_grasp.robot.moveit_planner import MoveItPlanner
    v=[.01] if degree>=3 else None
    a=[.002] if degree==5 else None
    plan=trajectory([point([0.],0.,v,a),point([.001],.1,v,a),
                     point([.008],1.,v,a)])
    desired=point([.06],v=[0.],a=[0.])
    with pytest.raises(ObservationVelocityLimitError) as error:
        validate(plan,desired,{'J1':.08})
    audit=error.value.audit
    assert audit['required_time_scale']>1.
    assert audit['trajectory_sha256']==wire_digest(plan)
    assert audit['limiting_segment']=='bridge' and audit['limiting_joint']=='J1'
    # Time-side headroom covers serialized nanosecond rounding; no velocity
    # threshold is enlarged and the final actual message is independently proved.
    scale=audit['required_time_scale']*(1.+1e-6)
    stretched,reason=MoveItPlanner._stretch_trajectory_timing(plan,scale)
    assert not reason
    report=validate(stretched,desired,{'J1':.08})
    assert report['required_time_scale']==1.
    assert report['trajectory_sha256']==wire_digest(stretched)!=wire_digest(plan)
    before=plan.joint_trajectory.points;after=stretched.joint_trajectory.points
    assert [p.positions for p in before]==[p.positions for p in after]
    old_bridge=segment_coefficients(desired,before[1],before[1].time_from_start.to_sec(),1)
    new_bridge=segment_coefficients(desired,after[1],after[1].time_from_start.to_sec(),1)
    np.testing.assert_allclose(new_bridge,old_bridge,atol=1e-8,rtol=0.)
    old_tail=segment_coefficients(before[1],before[2],.9,1)
    new_tail=segment_coefficients(after[1],after[2],
        (after[2].time_from_start-after[1].time_from_start).to_sec(),1)
    np.testing.assert_allclose(new_tail,old_tail,atol=1e-8,rtol=0.)


def test_unfinished_proof_is_not_a_retiming_authorization():
    from alicia_flexible_grasp.robot.observation_path_guard import ObservationVelocityLimitError
    plan=trajectory([point([0.],0.,[0.],[0.]),point([.1],1.,[0.],[0.])])
    with pytest.raises(ObservationPathError) as error:
        validate(plan,point([.06],v=[0.],a=[0.]),{'J1':.08},max_checks=0)
    assert not isinstance(error.value,ObservationVelocityLimitError)
    assert not hasattr(error.value,'audit')


def test_report_bounds_enclose_dense_random_cubic_and_quintic_speeds():
    from alicia_flexible_grasp.robot.observation_path_guard import (
        analyze_observation_trajectory_velocity, segment_coefficients, _polynomial,
    )
    rng=np.random.RandomState(804)
    for degree in (3,5):
        for _ in range(8):
            q=rng.uniform(-.03,.03,(3,2));v=rng.uniform(-.04,.04,(3,2))
            a=rng.uniform(-.04,.04,(3,2)) if degree==5 else [None]*3
            plan=trajectory([point(q[i],float(i),v[i],a[i]) for i in range(3)],['J1','J2'])
            desired=point([.01,-.01],v=[0.,0.],a=[0.,0.])
            report=analyze_observation_trajectory_velocity(plan,desired,{'J1':.08,'J2':.02})
            actual=np.zeros(2)
            for first,last in [(desired,plan.joint_trajectory.points[1]),
                               (plan.joint_trajectory.points[1],plan.joint_trajectory.points[2])]:
                c=segment_coefficients(first,last,1.,2)
                actual=np.maximum(actual,np.max(np.abs([
                    _polynomial(c[:,1:]*np.arange(1,6),s) for s in np.linspace(0.,1.,1001)]),axis=0))
            bounds=np.array([report['maximum_certified_speed_bound_rad_s'][n] for n in ['J1','J2']])
            assert np.all(actual<=bounds)
            assert report['required_time_scale']>=max(1.,float(np.max(actual/[.08,.02])))
