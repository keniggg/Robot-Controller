"""Reload the idle contact gateway, preserving manual mode and the recorded runtime identity."""
import argparse, hashlib, json, os, time, xmlrpc.client
from pathlib import Path

class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        c=super().make_connection(host); c.timeout=3.; return c

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--manifest',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--previous-baseline',type=Path,required=True);a=ap.parse_args()
    w=Path(__file__).resolve().parents[1]; mount=Path('/media/zhuyupei/SanStand');assert mount.is_mount() and mount in a.output.resolve().parents
    validation=json.loads(a.manifest.read_text());assert validation['passed'] is True and validation['reviewed'] is True
    previous=json.loads(a.previous_baseline.read_text());assert previous['deployment_passed'] is True
    def sources():
        for p,h in validation['source_sha256'].items():assert hashlib.sha256((w/p).read_bytes()).hexdigest()==h,p
    sources();a.output.mkdir(exist_ok=False);os.environ['ROS_LOG_DIR']=str(a.output/'ros')
    import rospy, rosgraph
    from std_msgs.msg import String,Header
    from sensor_msgs.msg import JointState
    from actionlib_msgs.msg import GoalStatusArray
    from alicia_flexible_grasp_supervisor.msg import GraspState
    rospy.init_node('reload_observation_view_gateway',anonymous=True,disable_rosout=True)
    master=rosgraph.Master(rospy.get_name())
    original_manual=rospy.get_param('/gui/joint_direct_mode');assert type(original_manual) is bool
    def rpc(n):return xmlrpc.client.ServerProxy(master.lookupNode(n),transport=Transport())
    def identity(n):
        response=rpc(n).getPid(rospy.get_name());assert response[0]==1
        pid=response[2];f=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split();assert f[0]!='Z'
        return dict(pid=pid,start_ticks=f[19],parent_pid=int(f[1]))
    def idle():
        assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5).active
        status=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=5).data)
        assert not(status.get('active') or status.get('start_reserved') or status.get('pending'))
        assert rospy.get_param('/gui/joint_direct_mode') is original_manual
        for t in ('/alicia_controller/follow_joint_trajectory/status','/hand_controller/follow_joint_trajectory/status'):
            msg=rospy.wait_for_message(t,GoalStatusArray,timeout=5);assert not any(g.status in (0,1,6,7) for g in msg.status_list)
    def feedback():
        q=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=5)
        assert q.header.frame_id=='sdk_measured' and 0 <= (rospy.Time.now()-q.header.stamp).to_sec()<.2
        e=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=5)
        status=rospy.wait_for_message('/alicia_d/actuation_status',String,timeout=5).data
        return dict(names=list(q.name),positions=list(q.position),epoch_ns=e.stamp.to_nsec(),status=status)
    target='/motion_gateway'
    keep=('/alicia_d_driver_node','/bessica_d_hw_interface','/move_group','/supervisor_camera','/remote_grasp6d_node','/grasp_task_node','/grasp_mode_router','/unknown_tabletop_perception','/compliant_gripper_controller','/alicia_supervisor_gui')
    r=dict(deployment_passed=False,motion_commands_sent=False,torque_disable_commands_sent=False,temperature_queried=False,source_sha256=validation['source_sha256'])
    def save():(a.output/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
    try:
        idle();r.update(before=identity(target),preserved_before={n:identity(n) for n in keep},feedback_before=feedback(),selection=rospy.get_param('/grasp_mode/selection'),manual_mode_preserved=original_manual,previous_runtime_baseline=str(a.previous_baseline));save()
        assert r['before']==previous['preserved_after'][target]
        expected=dict(previous['preserved_after'],**previous['after'])
        assert all(row==expected[name] for name,row in r['preserved_before'].items())
        # shutdown may close its own XML-RPC reply socket. Send it only once,
        # then prove the old process exited and the same roslaunch owner respawned.
        r['shutdown_requested']=True; save()
        try:
            reply=rpc(target).shutdown(rospy.get_name(),
                'load reviewed held-approach precheck parity while task idle')
            assert reply[0]==1
            r['shutdown_reply_received']=True
        except (ConnectionError, OSError, xmlrpc.client.ProtocolError) as exc:
            r['shutdown_reply_unavailable']=repr(exc)
        save()
        until=time.monotonic()+45
        while time.monotonic()<until:
            try:
                after=identity(target)
                if after['pid']!=r['before']['pid'] and after['parent_pid']==r['before']['parent_pid']:break
            except Exception:pass
            time.sleep(.2)
        else:raise RuntimeError('existing gateway owner did not respawn')
        old_process=Path('/proc')/str(r['before']['pid'])/'stat'
        assert not old_process.exists() or old_process.read_text().rsplit(')',1)[1].split()[19]!=r['before']['start_ticks'] or old_process.read_text().rsplit(')',1)[1].split()[0]=='Z'
        rospy.wait_for_service('/supervisor/check_contact_sequence',timeout=25)
        idle();sources();r['after']=after;r['feedback_after']=feedback();r['preserved_after']={n:identity(n) for n in keep}
        assert r['preserved_before']==r['preserved_after']
        assert r['feedback_before']['epoch_ns']==r['feedback_after']['epoch_ns']
        assert r['feedback_before']['names']==r['feedback_after']['names']
        assert max(abs(x-y) for x,y in zip(r['feedback_before']['positions'],r['feedback_after']['positions']))<=.001534
        assert rospy.get_param('/grasp_mode/selection')==r['selection']
        r['deployment_passed']=True
    except Exception as e:r['error']=repr(e);raise
    finally:save();print(json.dumps(dict(deployment_passed=r['deployment_passed'],output=str(a.output),error=r.get('error')),ensure_ascii=False),flush=True)
if __name__=='__main__':main()
