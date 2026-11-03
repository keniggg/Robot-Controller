"""Load tested numeric decode timing into idle perception only."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
import xmlrpc.client

import rosgraph
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_flexible_grasp_supervisor.srv import TriggerZero

WORK = Path(__file__).resolve().parents[1]
SESSION = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/supplement_grasp_20261008T103117Z')
TARGET = '/unknown_tabletop_perception'
PRESERVE = ('/alicia_d_driver_node', '/motion_gateway', '/grasp_task_node',
            '/supervisor_camera', '/bessica_d_hw_interface', '/move_group',
            '/grasp_mode_router', '/compliant_gripper_controller', '/remote_grasp6d_node')


class Transport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host); connection.timeout = 4.; return connection


def main():
    assert Path('/media/zhuyupei/SanStand').is_mount()
    validation = json.loads((SESSION/'verification/decode_timing/validated_sources.json').read_text())
    assert validation['passed'] is True
    def source_check():
        for path,digest in validation['source_sha256'].items():
            assert hashlib.sha256((WORK/path).read_bytes()).hexdigest() == digest, path
    source_check()
    out = SESSION/'decode_timing_deployment'; out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR'] = str(out/'observer_ros')
    rospy.init_node('load_decode_timing_perception', anonymous=True, disable_rosout=True)
    master = rosgraph.Master(rospy.get_name())
    def proxy(name): return xmlrpc.client.ServerProxy(master.lookupNode(name),transport=Transport())
    def identity(name):
        pid=proxy(name).getPid(rospy.get_name())[2]
        return dict(pid=pid,start_ticks=(Path('/proc')/str(pid)/'stat').read_text().rsplit(')',1)[1].split()[19])
    def idle():
        assert not rospy.wait_for_message('/grasp/state',GraspState,timeout=5.).active
        status=json.loads(rospy.wait_for_message('/grasp_mode/status',String,timeout=5.).data)
        assert status.get('active') is False and status.get('start_reserved') is False and not status.get('pending')
        assert rospy.get_param('/gui/joint_direct_mode') is False
        assert not rospy.get_param('/rgb_acquisition/ledger',{})
    def measured():
        q=rospy.wait_for_message('/alicia_d/accepted_joint_states',JointState,timeout=5.)
        assert q.header.frame_id=='sdk_measured' and 0 <= (rospy.Time.now()-q.header.stamp).to_sec() < .2
        return dict(names=list(q.name),positions=list(q.position))
    def epoch():
        m=rospy.wait_for_message('/alicia_d/control_reference_epoch',Header,timeout=5.)
        assert m.frame_id=='sdk_reference_epoch'; return m.stamp.to_nsec()
    result=dict(deployment_passed=False,restarted_nodes=[TARGET],motion_commands_sent=False,
                joint_disable_commands_sent=False,temperature_queried=False,
                validated_source_sha256=validation['source_sha256'])
    try:
        idle(); selection=rospy.get_param('/grasp_mode/selection')
        before={n:identity(n) for n in (TARGET,)+PRESERVE}; q0=measured(); e0=epoch()
        result.update(before=before,selection=selection,measured_before=q0,epoch_before=e0)
        response=rospy.ServiceProxy('/grasp_6d/request_plan',TriggerZero)(False)
        assert response.success,response.message
        idle();source_check()
        assert rospy.get_param('/grasp_mode/selection')==selection and epoch()==e0
        assert identity(TARGET)==before[TARGET]
        reply=proxy(TARGET).shutdown(rospy.get_name(),'load validated diagnostic-only decode timing while idle')
        assert reply[0]==1,reply
        old=Path('/proc')/str(before[TARGET]['pid'])/'stat'; deadline=time.monotonic()+15.
        while old.exists() and old.read_text().rsplit(')',1)[1].split()[0]!='Z':
            assert time.monotonic()<deadline,'old planner did not exit';time.sleep(.1)
        unit='alicia-decode-timing-perception-20261008'
        command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec',
            '--property=Restart=no','--property=WorkingDirectory='+str(WORK),
            '--property=StandardOutput=append:'+str(out/'perception.log'),
            '--property=StandardError=append:'+str(out/'perception.log'),
            '--setenv=ROS_LOG_DIR='+str(out/'perception_ros'),'/bin/bash',
            str(WORK/'.runtime/start_decode_timing_perception_20261008.sh')]
        result.update(unit=unit,command=command);subprocess.run(command,check=True,timeout=15.)
        deadline=time.monotonic()+30.
        while True:
            try:
                if identity(TARGET)!=before[TARGET]: break
            except Exception: pass
            assert time.monotonic()<deadline,'new planner unavailable';time.sleep(.2)
        rospy.wait_for_service('/grasp_mode/reset_remote',timeout=25.)
        reset=rospy.ServiceProxy('/grasp_mode/reset_remote',Trigger)()
        assert reset.success,reset.message
        result['remote_reset']=dict(success=reset.success,message=reset.message)
        after={n:identity(n) for n in (TARGET,)+PRESERVE}
        assert all(after[n]==before[n] for n in PRESERVE)
        assert after[TARGET]!=before[TARGET]
        idle(); assert rospy.get_param('/grasp_mode/selection')==selection and epoch()==e0
        q1=measured();assert q0==q1,'measured posture changed';source_check()
        result.update(deployment_passed=True,after=after,measured_after=q1)
    except Exception as exc:
        result['error']=repr(exc);raise
    finally:
        (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__': main()
