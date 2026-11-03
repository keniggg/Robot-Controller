"""New-target selection from this turn's explicit alignment; no old-episode retry."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_grasp_modes.srv import SetGraspMode

WORK = Path(__file__).resolve().parents[1]
MOUNT = Path('/media/zhuyupei/SanStand')
RUN = None
spec=importlib.util.spec_from_file_location('alignment_contract',WORK/'tools/diagnostics/prepare_rgb_acquisition_retry.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)


def main():
    assert MOUNT.is_mount(), 'SanStand unavailable; no fallback'
    index=json.loads((WORK/'.runtime/current_alignment_supplement_view_20261006.json').read_text())
    receipt_path=Path(index['receipt']);receipt=json.loads(receipt_path.read_text())
    RUN = receipt_path.parent
    os.environ["ROS_LOG_DIR"] = str(RUN/"ros")
    assert receipt['confirmation_id']=='call_bxnlc8gAuMDZZirhSJaHItcz:0'
    assert receipt['confirmation_source']['text']=='已调整并对准，可以继续验证'
    rospy.init_node('select_current_confirmed_new_target',anonymous=True,disable_signals=True)
    task=rospy.wait_for_message('/grasp/state',GraspState,timeout=4)
    assert not task.active, 'existing task owns motion'
    ledger=rospy.get_param('/rgb_acquisition/ledger',{})
    assert not ledger, 'explicit realignment selection requires no active observation episode'
    epoch=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4)
    msg=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=4)
    assert msg.header.frame_id=='sdk_measured'
    values=dict(zip(msg.name,msg.position));q=np.array([values[n] for n in guard.JOINT_NAMES])
    assert np.isfinite(q).all() and np.max(np.abs(q-np.array(receipt['bound_feedback']['positions'])))<=guard.POSE_TOLERANCE_RAD
    now=rospy.Time.now().to_nsec()
    assert int(receipt['bound_feedback']['stamp_ns'])<=msg.header.stamp.to_nsec()<=now
    guard.require_fresh_source(msg.header.stamp.to_nsec(),now)
    context=guard.retry_source_context(rospy.get_param('/grasp_mode/selection'),epoch.stamp.to_nsec(),
        rospy.get_param('/camera'),rospy.get_param('/handeye'),rospy.get_param('/robot_description'))
    assert guard.digest(context)==receipt['source_context_sha256'], 'alignment context changed'
    rospy.wait_for_service('/grasp_mode/select',timeout=4)
    report=dict(path=str(RUN),preview_only=False,alignment_receipt=receipt,
        alignment_receipt_file=str(receipt_path),alignment_receipt_sha256=hashlib.sha256(receipt_path.read_bytes()).hexdigest(),
        previous_selection=context['selection'],previous_ledger=ledger,new_selection=None,
        reason='Continue under the original unchanged user alignment after verified planner repair; create fresh measurements without reusing the failed plan',
        selection_kind='fresh_measurement_after_verified_planner_repair',motion_commands_sent=False,torque_commands_sent=False)
    try:
        response=rospy.ServiceProxy('/grasp_mode/select',SetGraspMode)('unknown','observe_return')
        report['response']=dict(success=response.success,message=response.message)
        assert response.success, response.message
        until=time.monotonic()+4
        while time.monotonic()<until:
            new=guard.parse_selection(rospy.get_param('/grasp_mode/selection'))
            if new['generation']==context['selection']['generation']+1 and new['stamp_ns']>context['selection']['stamp_ns']:
                assert new['mode']=='unknown' and new['strategy']=='observe_return'
                report['new_selection']=new;break
            time.sleep(.05)
        assert report['new_selection'] is not None, 'selection acknowledgement missing'
    except Exception as exc:
        report['failure']=repr(exc)
        raise
    finally:
        path=RUN/'new_target_selection.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(dict(path=str(path),selection=report['new_selection'],failure=report.get('failure')),ensure_ascii=False),flush=True)


if __name__=='__main__':main()
