"""Persist task facts without replacing unrelated historical results."""
from datetime import datetime,timezone
import json
from pathlib import Path
WORK=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN=EXT/'ros_log/full_ros_resume_20261004T002930Z'
if not EXT.parent.parent.is_mount():
    raise RuntimeError('SanStand not mounted')
for path in (RUN/'current_progress.json',WORK/'.runtime/latest_full_ros_start.json'):
    data=json.loads(path.read_text())
    data.update(updated_at_utc=datetime.now(timezone.utc).isoformat(),
        current_selection_generation=10,current_generation_target_lost=True,
        current_generation_motion_commands_sent=False,grasp_commands_sent_this_turn=False,
        storage_blocked=False,task_complete=False,
        latest_rgb_acquisition_deployment=str(RUN/'rgb_acquisition_deployment_20261004T075537Z.json'),
        loaded_joint_regression='2167 passed, 1 existing skipped; deployed multi-fragment boundary implementation',
        pending_offline_regression='2222 passed, 1 existing skipped; source bd8142; not loaded yet',
        driver_temperature_policy='software monitoring false, poll 0; queries and software reactions disabled by user instruction',
        driver_temperature_policy_deployment=str(RUN/'driver_temperature_off_20261004T084125Z/result.json'),
        driver_pid=133511, driver_control_reference_epoch_ns='1791103287611436565',
        driver_feedback_ready=True, driver_actuation_status='PENDING:POSITIVE_ENABLE_REQUESTED',
        actual_directional_actuation_confirmation_pending=True,
        fragment_fix_raw_capture=str(EXT/'ros_log/rgb_observation_bootstrap_20261004T073256Z/result.json'),
        latest_readonly_original_intent_verification=str(EXT/'ros_log/rgb_observation_new_trial_20261004T084737.784476Z/result.json'),
        current_blocker='Storage available. Gen10 exact loss source has a 200.04325 ms SDK bracket from the diagnostic poll slot, plus a genuine fragment ownership conflict on exact replay. Temperature-off driver deployed per explicit user request; post-reload 301 accepted feedback, max gap100.117022ms, codes06 00 only. Six other core nodes preserved, no disable sent. Real actuation remains pending (positive enable request is not confirmation). Pending-source producer passed2222 tests but not loaded. Old gen10 lost is not revived. Fresh RGB/SDK after real new epoch directly matches original user-confirmed RGB via recorded maintenance edge, read-only qualification passed. Preparing tested dispatch/load for a new selection, no grasp success claimed.')
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
print('Progress saved; no ROS actions')
