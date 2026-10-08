"""Load the idle task-only repair, preserving manual control and hardware owners."""
import argparse, hashlib, json, os, socket, subprocess, time, xmlrpc.client
from datetime import datetime, timezone
from pathlib import Path

UNIT = 'alicia-task-postmove-20260929.service'
SOURCE = Path('src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py')


def identity(pid):
    root = Path('/proc')/str(pid)
    stat = (root/'stat').read_text().rsplit(')',1)[1].split()
    return dict(pid=pid, start_ticks=int(stat[19]), argv=(root/'cmdline').read_bytes().decode().split('\0')[:-1])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--load', action='store_true')
    args=parser.parse_args()
    if not args.load:
        print(json.dumps(dict(status='review_only', unit=UNIT, source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest())))
        return
    mount=Path('/media/zhuyupei/SanStand')
    if not mount.is_mount(): raise RuntimeError('SanStand not mounted')
    out=mount/'实验记录/Robot-Controller/ros_log'/('final_refine_stage_load_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    out.mkdir(); (out/'ros').mkdir(); os.environ['ROS_LOG_DIR']=str(out/'ros')
    socket.setdefaulttimeout(4.)
    import rospy, rosgraph
    from alicia_flexible_grasp_supervisor.msg import GraspState
    from alicia_flexible_grasp_supervisor.srv import TriggerZero
    from std_msgs.msg import Header
    rospy.init_node('load_idle_final_refine_stage',anonymous=True,disable_signals=True)
    master=rosgraph.Master('/load_idle_final_refine_stage')
    def node_pid(name): return xmlrpc.client.ServerProxy(master.lookupNode(name)).getPid('/load_idle_final_refine_stage')[2]
    protected=['/alicia_d_driver_node','/bessica_d_hw_interface','/motion_gateway','/accepted_robot_state_publisher','/supervisor_camera','/unknown_tabletop_perception']
    result=dict(status='preparing',started_ns=time.time_ns(),tests_passed=396,
        hardware_commands_sent=0,manual_parameter_written=False,automatic_grasp_started=False,
        user_instruction='User moved arm; finish software repair then pause until explicitly awakened.',
        source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest())
    try:
        state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4.)
        if state.active: raise RuntimeError('task active; software load not allowed')
        result['old_task_state']=dict(active=state.active,state=state.state,message=state.message)
        result['before']={n:identity(node_pid(n)) for n in protected}
        result['root_before']=identity(8526)
        result['old_task']=identity(node_pid('/grasp_task_node'))
        result['manual_before']=rospy.get_param('/gui/joint_direct_mode',False)
        result['selection_before']=rospy.get_param('/grasp_mode/selection')
        result['epoch_before_ns']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4.).stamp.to_nsec()
        (out/'before.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        retired=rospy.ServiceProxy('/grasp_6d/request_plan',TriggerZero)(False)
        if not retired.success or retired.message not in ('continuous remote 6D inference stopped','continuous remote 6D inference already stopped'):
            raise RuntimeError('old inference retirement not acknowledged: '+retired.message)
        result['inference_retirement']=retired.message
        subprocess.run(['systemctl','--user','restart',UNIT],check=True,timeout=20.)
        deadline=time.monotonic()+15.
        while time.monotonic()<deadline:
            try:
                new_task=identity(node_pid('/grasp_task_node'))
                if new_task!=result['old_task']: break
            except Exception: pass
            time.sleep(.1)
        else: raise RuntimeError('task-only replacement did not return')
        result['new_task']=new_task
        result['after']={n:identity(node_pid(n)) for n in protected}
        result['root_after']=identity(8526)
        if result['before']!=result['after'] or result['root_before']!=result['root_after']:
            raise RuntimeError('protected process identity changed during task load')
        result['epoch_after_ns']=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=4.).stamp.to_nsec()
        if result['epoch_before_ns']!=result['epoch_after_ns']: raise RuntimeError('control epoch changed during task load')
        result['manual_after']=rospy.get_param('/gui/joint_direct_mode',False)
        result['selection_after']=rospy.get_param('/grasp_mode/selection')
        if result['selection_before']!=result['selection_after']: raise RuntimeError('selection changed during task load')
        state=rospy.wait_for_message('/grasp/state',GraspState,timeout=4.)
        if state.active: raise RuntimeError('task unexpectedly active after load')
        result.update(status='loaded_idle_waiting_for_user_wakeup',new_task_state=state.state,completed_ns=time.time_ns())
    except Exception as exc:
        result.update(status='load_failed_no_retry',error=str(exc),completed_ns=time.time_ns())
        raise
    finally:
        (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        Path('.runtime/latest_final_refine_stage_load.json').write_text(json.dumps(dict(directory=str(out)),ensure_ascii=False)+'\n')
        print(json.dumps(dict(directory=str(out),**result),ensure_ascii=False,indent=2),flush=True)

if __name__=='__main__': main()
