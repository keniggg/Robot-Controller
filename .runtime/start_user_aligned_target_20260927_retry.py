import json
import os
from pathlib import Path
import subprocess
import sys
import time
import rospy
from std_msgs.msg import Bool, String
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_grasp_modes.srv import SetGraspMode

rospy.init_node('current_unknown_grasp_handoff', anonymous=True)
task = rospy.wait_for_message('/grasp/state', GraspState, timeout=5)
if task.active:
    raise RuntimeError('Existing task owns execution')
strategy = os.environ.get('GRASP_STRATEGY', 'observe_return')
if strategy not in ('two_stage', 'direct', 'observe_return'):
    raise ValueError('Unsupported grasp strategy')
selected = rospy.get_param('/grasp_mode/selection')
if selected['mode'] != 'unknown' or selected['strategy'] not in ('two_stage', 'direct', 'observe_return'):
    raise RuntimeError('Current selection changed; refusing a different task')
rospy.wait_for_service('/grasp_mode/select', timeout=5)
result = rospy.ServiceProxy('/grasp_mode/select', SetGraspMode)('unknown', strategy)
print('FRESH_TARGET_SELECTION', result.success, result.message, flush=True)
if not result.success or result.message != 'SWITCHING':
    raise RuntimeError('Fresh target selection failed')
cutoff = time.monotonic() + 35
while time.monotonic() < cutoff:
    status = json.loads(rospy.wait_for_message('/grasp_mode/status', String, timeout=3).data)
    if status.get('generation', 0) > selected['generation'] and status.get('state') == 'ready':
        break
else:
    print('CANDIDATE_COMPUTATION_START success=False message=TARGET_NOT_READY ' + json.dumps(status), flush=True)
    sys.exit(2)
selection = rospy.get_param('/grasp_mode/selection')
if selection['mode'] != 'unknown' or selection['strategy'] != strategy or selection['generation'] != status['generation']:
    raise RuntimeError('Selection changed during target acquisition')
rospy.set_param('/gui/joint_direct_mode', False)
pub = rospy.Publisher('/gui/joint_direct_mode', Bool, queue_size=1, latch=True)
rospy.sleep(.3)
pub.publish(Bool(data=False))
rospy.sleep(.3)
record = {'user_request': '现在已经对准了目标物，重新执行6D抓取任务', 'target_selection': 'current central isolated tabletop object',
          'mode_selection': selection, 'handoff_unix': time.time(), 'mass_estimate_added': False,
          'torque_command_sent': False, 'recording_scope': 'local only'}
if os.environ.get('GRASP_OPERATOR_MASS_KG'):
    import math
    estimate = float(os.environ['GRASP_OPERATOR_MASS_KG'])
    statement = os.environ['GRASP_OPERATOR_MASS_STATEMENT']
    if not math.isfinite(estimate) or estimate <= 0:
        raise ValueError('Invalid operator mass estimate')
    # The router-ready and detector topics arrive independently. Wait for
    # the detector's matching generation instead of consuming an old latch.
    target_deadline = time.monotonic() + 10.0
    while time.monotonic() < target_deadline:
        target = json.loads(rospy.wait_for_message('/perception/unknown/detector_status', String, timeout=3).data)
        if (target.get('state') == 'ready' and target.get('target_locked') and not target.get('target_lost')
                and target.get('generation') == selection['generation']
                and str(target.get('selection_stamp_ns')) == str(selection['stamp_ns'])
                and rospy.get_param('/grasp_mode/selection') == selection):
            break
        rospy.sleep(.1)
    else:
        raise RuntimeError('Current target unavailable for mass binding: '+json.dumps(target))
    now_ns = rospy.Time.now().to_nsec()
    evidence = dict(selection=selection, source='operator_estimate', operator_statement=statement,
        estimated_mass_kg=estimate, anchor_source_stamp_ns=str(target['anchor_source_stamp_ns']),
        reported_at_ns=str(now_ns), expires_at_ns=str(now_ns + 1200 * 1000000000))
    rospy.set_param('/grasp_mode/operator_mass_estimate', evidence)
    record['operator_mass_evidence'] = evidence
    record['mass_estimate_added'] = True
(Path(os.environ['GRASP_RECORD_DIR']) / 'authorization.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
print('CURRENT_TARGET_READY ' + json.dumps(record, ensure_ascii=False), flush=True)
sys.exit(subprocess.call(['rosrun','alicia_grasp_modes','mode_aware_grasp_runner.py','--mode','unknown','--strategy',strategy]))
