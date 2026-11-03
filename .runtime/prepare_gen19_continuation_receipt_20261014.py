"""Record the actual resume/site reply and a just-reviewed scene; no ROS calls."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import uuid
from continuous_grasp_authorization_20261012 import atomic_json

M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen18_restart_20261014T012216Z'
A = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/authorization.json'
def read(p): return json.loads(Path(p).read_text())
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def ref(p): return dict(path=str(p),sha256=sha(p))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scene',type=Path,required=True)
    ap.add_argument('--readiness',type=Path,required=True)
    ap.add_argument('--visual-assessment',required=True)
    args=ap.parse_args();s=args.scene.resolve();ready=args.readiness.resolve()
    assert M.is_mount() and not os.statvfs(M).f_flag & os.ST_RDONLY
    assert N in s.parents and N in ready.parents
    assert sha(A)=='132e83f926a249f8d1294ddeb74e8b38224744ea369a7acf8a6c29802f0c9172'
    auth=read(A);hands=read(auth['original_hands_clear_receipt']['path'])
    pre=read(s/'preflight.json');r=read(ready);now=time.time_ns()
    assert r['passed'] and r['maximum_joint_delta_rad']==0.0 and r['selection']==pre['selection']
    assert pre['receipt_ns']<=r['recorded_ns']<=now and now-pre['receipt_ns']<90_000_000_000
    assert not (s/'operator_observation.json').exists()
    clear=dict(source='actual_user_question_reply',message_id='call_834864bda3674a29b0e2a72195aa0218:0',user_message='已停止操作，双手及前臂均已移到活动范围之外',recorded_ns=now,recorded_time_kind='locally_recorded_receipt_time',new_grasp_authorization_claimed=False)
    atomic_json(s/'scene_clear_receipt.json',clear)
    resume=dict(source='actual_user_message',user_text='继续处理',platform_message_id=None,local_record_id=uuid.uuid4().hex,recorded_ns=now,recorded_time_kind='locally_recorded_receipt_time',continuous_authorization=ref(A),scope='Resume unfinished repair and continuous recorded grasp after latest pause; site clearance separately confirmed.')
    atomic_json(s/'resume_request.json',resume)
    ob=dict(version=1,observation_id=uuid.uuid4().hex,new_user_confirmation_claimed=False,same_target_observed=True,target_visible=True,hands_clear_basis=dict(prior_explicit_user_reply_id=hands['message_id'],current_image_review_no_conflicting_hand_in_motion_area=True,current_scene_clear_reply=ref(s/'scene_clear_receipt.json')),image=dict(ref(s/'current_scene.png'),source_stamp_ns=pre['image_stamp_ns']),preflight=ref(s/'preflight.json'),readiness=ref(ready),captured_ns=pre['receipt_ns'],assessed_ns=now,valid_until_ns=now+120_000_000_000,visual_assessment=args.visual_assessment,current_resume_request=ref(s/'resume_request.json'),preparation_script_sha256=sha(__file__))
    atomic_json(s/'operator_observation.json',ob)
    with (A.parent/'control.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX);c=read(A.parent/'control.json')
        assert c['state']=='paused' and c['revision']==13
        c.update(state='active',revision=14,updated_ns=now,resumed_ns=now,resume_user_message='继续处理',resume_session=str(N),resume_evidence=str(s/'resume_request.json'),resume_scope='Continue recorded grasp until physical success; latest site clearance separately recorded.',pause_scope='Historical pause superseded by latest explicit resume.')
        atomic_json(A.parent/'control.json',c)
    print(json.dumps(dict(observation=str(s/'operator_observation.json'),control_state='active',revision=14,valid_until_ns=ob['valid_until_ns']),ensure_ascii=False))

if __name__=='__main__': main()
