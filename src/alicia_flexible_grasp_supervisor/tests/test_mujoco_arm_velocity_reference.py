"""Physical actuator-force contracts for prescribed arm motion."""
from copy import deepcopy
from types import SimpleNamespace as NS

import numpy as np
import pytest

from test_mujoco_digital_twin_server_protocol import server_module as engine


def plant():
    # Arm joints/actuators deliberately noncontiguous; index 1 is a gripper.
    model = NS(jnt_qposadr=np.arange(3), jnt_dofadr=np.arange(3),
        actuator_gainprm=np.array([[70.]+[0.]*9, [200.]+[0.]*9, [90.]+[0.]*9]),
        actuator_biasprm=np.array([[0.,-70.,-10.]+[0.]*7,
                                  [0.,-200.,-7.]+[0.]*7,
                                  [0.,-90.,-12.]+[0.]*7]),
        actuator_gear=np.tile([1.,0.,0.,0.,0.,0.],(3,1)),
        actuator_trnid=np.array([[2,-1],[1,-1],[0,-1]]),
        actuator_dyntype=np.zeros(3,dtype=int), actuator_gaintype=np.zeros(3,dtype=int),
        actuator_biastype=np.ones(3,dtype=int), actuator_trntype=np.zeros(3,dtype=int),
        actuator_ctrllimited=np.zeros(3,dtype=bool), actuator_ctrlrange=np.zeros((3,2)))
    data = NS(qpos=np.array([.1,.008,-.2]), qvel=np.array([.3,.017,-.4]),
              ctrl=np.array([.4,.019,-.3]))
    backend = engine.MujocoDigitalTwinBackend()
    backend._mujoco=NS(mj_forward=lambda model,data:None)
    return backend,model,data


@pytest.mark.parametrize('velocities',[(.07,-.05),(-.07,.05),(0.,0.)])
def test_existing_pd_force_acts_on_velocity_error_not_requested_speed(velocities):
    backend,model,data=plant(); before=deepcopy(model.__dict__)
    start=np.array([.1,-.2]); velocity=np.array(velocities); target=start+.002*velocity
    gap_state=(data.qpos[1],data.qvel[1],data.ctrl[1])
    returned=backend._apply_velocity_consistent_arm_reference(
        model,data,[0,2],[2,0],start,target,.002)
    for j,a,k in ((0,2,0),(2,0,1)):
        kp=model.actuator_gainprm[a,0]; kv=-model.actuator_biasprm[a,2]
        # Independent actuator-force equation from MuJoCo's affine actuator.
        for measured_velocity in (velocity[k], velocity[k]-.011,velocity[k]+.009):
            actual_force=kp*data.ctrl[a]-kp*data.qpos[j]-kv*measured_velocity
            desired_force=kp*(target[k]-start[k])+kv*(velocity[k]-measured_velocity)
            assert actual_force == pytest.approx(desired_force,abs=1e-12)
        if velocity[k]==0.:
            assert data.ctrl[a]==target[k]
    assert returned == pytest.approx(max(abs(velocity)))
    assert gap_state==(data.qpos[1],data.qvel[1],data.ctrl[1])
    for key,value in before.items():
        np.testing.assert_array_equal(model.__dict__[key],value)


@pytest.mark.parametrize('field,index,value',[
    ('actuator_gainprm',(0,0),0.), ('actuator_gainprm',(0,0),float('nan')),
    ('actuator_biasprm',(0,2),1.), ('actuator_biasprm',(0,1),-71.),
    ('actuator_gear',(0,0),2.), ('actuator_trnid',(0,0),1),
    ('actuator_gaintype',0,1), ('actuator_biastype',0,0),
    ('actuator_dyntype',0,1), ('actuator_trntype',0,1),
])
def test_unsupported_second_actuator_rejects_entire_step_before_state_mutation(field,index,value):
    backend,model,data=plant();before=deepcopy(data.__dict__)
    getattr(model,field)[index]=value
    with pytest.raises(RuntimeError):
        backend._apply_velocity_consistent_arm_reference(model,data,[0,2],[2,0],[.1,-.2],[.11,-.21],.02)
    for key,value in before.items():np.testing.assert_array_equal(data.__dict__[key],value)


def test_ctrl_limit_is_enforced_without_clamping_or_changing_force_limits():
    backend,model,data=plant();before=deepcopy(data.__dict__)
    model.actuator_ctrllimited[0]=True;model.actuator_ctrlrange[0]=[-.2,.2]
    with pytest.raises(RuntimeError,match='control range'):
        backend._apply_velocity_consistent_arm_reference(model,data,[0,2],[2,0],[.1,-.2],[.11,-.21],.02)
    for key,value in before.items():np.testing.assert_array_equal(data.__dict__[key],value)


@pytest.mark.parametrize('start,target,dt',[(float('nan'),.1,.002),(.1,float('inf'),.002),(.1,.2,0.)])
def test_nonfinite_motion_never_mutates_physical_state(start,target,dt):
    backend,model,data=plant();before=deepcopy(data.__dict__)
    with pytest.raises(RuntimeError):
        backend._apply_velocity_consistent_arm_reference(model,data,[0],[2],[start],[target],dt)
    for key,value in before.items():np.testing.assert_array_equal(data.__dict__[key],value)


def test_real_mujoco_reports_position_error_force_at_matching_requested_velocity():
    mj=pytest.importorskip('mujoco')
    model=mj.MjModel.from_xml_string('''<mujoco><option gravity="0 0 0"/>
      <worldbody><body><joint name="a" type="slide"/>
      <geom type="sphere" size=".01" mass="1"/></body></worldbody>
      <actuator><position name="a_act" joint="a" kp="90" kv="12" forcerange="-5 5"/></actuator>
      </mujoco>''')
    data=mj.MjData(model);backend=engine.MujocoDigitalTwinBackend();backend._mujoco=mj
    force_range=model.actuator_forcerange.copy()
    backend._apply_velocity_consistent_arm_reference(model,data,[0],[0],[.1],[.10014],.002)
    assert data.actuator_force[0] == pytest.approx(90.*.00014,abs=1e-12)
    np.testing.assert_array_equal(model.actuator_forcerange,force_range)
