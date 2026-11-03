"""Preserve and update the local-only experiment status without touching ROS."""
from datetime import datetime, timezone
import json
from pathlib import Path

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
MOUNT = Path('/media/zhuyupei/SanStand')
RUN = MOUNT/'实验记录/Robot-Controller/ros_log/full_ros_resume_20261004T002930Z'
if not MOUNT.is_mount():
    raise RuntimeError('SanStand unavailable; no fallback')
now = datetime.now(timezone.utc)
for path in (RUN/'current_progress.json', WORK/'.runtime/latest_full_ros_start.json'):
    value = json.loads(path.read_text())
    backup = RUN/(path.stem+'_before_progress_'+now.strftime('%Y%m%dT%H%M%SZ')+'.json')
    backup.write_text(path.read_text())
    value.update(updated_at_utc=now.isoformat(), task_complete=False,
        historical_grasp_too_high_resolved=True, current_cross_view_fusion_resolved=False,
        current_initial_target_acquired=False, robot_movement_commands_sent_this_turn=True,
        torque_disable_commands_sent=False,
        rgb_acquisition_execution_integrated=True, rgb_acquisition_execution_loaded=True,
        rgb_interest_set_integration='loaded_real_observation_completed_target_not_acquired',
        observation_only_bootstrap_available_without_measured_target=True,
        grasp_commands_sent_this_turn=False,
        latest_rgb_acquisition_deployment=str(RUN/'rgb_acquisition_deployment_20261004T063056Z.json'),
        latest_live_attempts=[str(RUN.parent/('rgb_acquisition_run_'+stamp)/'result.json')
            for stamp in ('20261004T033850Z','20261004T035637Z','20261004T042301Z','20261004T044159Z','20261004T045612Z','20261004T052821Z','20261004T060311Z')],
        completed_observation_count=6, intended_target_observation_count=4,
        wrong_rgb_interest_observation_count=2, automatic_motion_paused_for_user_manual_adjustment=False,
        user_alignment_confirmed=True, current_selection_generation=8,
        current_generation_motion_commands_sent=False,
        current_generation_target_lost=True,
        latest_alignment_receipt=str(RUN/'alignment_receipt_20261004T061353Z.json'),
        latest_measured_support_difference=dict(height_mm=1.7711, angle_deg=.2824,
            scope='trial7 latest handoff fixed-footprint audit only; not historical wide-pose validation'),
        current_blocker='Original-reference FOV fix is tested (1926 pass/1 existing skip) and '
            'loaded; actual prior trajectory is now rejected at 4 px versus unchanged 36 px. '
            'New confirmed alignment established generation8. Full grasp recorder refused '
            'because SanStand free=1297481728 bytes versus required=4831838208 bytes; child '
            'grasp runner never started and generation8 sent no motion/gripper command. User '
            'storage clarification pending; no files deleted or recording gate lowered. '
            'Generation8 briefly acquired target then lost on a single raw depth fragment '
            '(XY extent ratio .48445/.51236, support .0912 mm/.0498 deg). Adding bounded '
            'identity-only hold for strictly contained shrinking fragment, without geometry '
            'authority or recovery of current lost latch. Historical wide-pose 19-20mm '
            'inconsistency and actual thin-target grasp remain incomplete.',
        storage_blocked=True, storage_available_bytes_at_grasp_admission=1297481728,
        storage_required_bytes=4831838208,
        latest_joint_regression='1926 passed, 1 pre-existing local fixture skipped',
        loaded_joint_regression='1926 passed, 1 pre-existing local fixture skipped')
    path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
print('Progress preserved and updated on SanStand and project index.')
