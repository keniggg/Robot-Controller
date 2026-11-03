"""One bounded offline manual-revocation diagnostic; no pytest suite or ROS initialization."""
import ast
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

W=Path(__file__).resolve().parents[1]
C=W/'.runtime/gen43_mask_candidate'
Q=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/verification')
O=Path(sys.argv[1]).resolve()
assert Q.parents[4].is_mount() and O.parent==Q and not O.exists()
O.mkdir();(O/'case').mkdir()
os.environ['ROS_LOG_DIR']=str(O/'ros')
sys.path.append(str(W/'devel/lib/python3/dist-packages'))
for path in reversed([C/'src/alicia_flexible_grasp_supervisor/src',C/'src/alicia_grasp_modes/src',C/'src/alicia_flexible_grasp_supervisor/tests',C/'src/alicia_grasp_modes/tests']):
    sys.path.insert(0,str(path))
from _pytest.monkeypatch import MonkeyPatch
import test_gen43_top_mask_handoff as fixture
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
source_files=[Path(fixture.__file__),Path(__file__).resolve(),
    C/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py',
    C/'src/alicia_flexible_grasp_supervisor/tests/test_gen43_top_mask_contract.py',
    C/'src/alicia_grasp_modes/tests/test_gen35_minimal_chain_replay.py']
source_sha={str(p):sha(p) for p in source_files}
patch=MonkeyPatch();fixture.no_unsimulated_boundaries.__wrapped__(patch)
contexts=[];original=fixture.chain.unified_replay
def capture(*args,**kwargs):
    context=original(*args,**kwargs);contexts.append(context);return context
patch.setattr(fixture.chain,'unified_replay',capture)
r=dict(passed=False,kind='single_original_chain_manual_revocation_causality',
    pytest_suite_run=False,directed_new_case_function_invoked=True,
    ROS_initialized=False,hardware_accessed=False,motion_commands_sent=False,temperature_queried=False,
    source_sha256=source_sha)
try:
    fixture.test_qualified_top_mask_never_bypasses_real_control_or_target_revocation(patch,O/'case','manual_control')
    assert len(contexts)==1
    context=contexts[0]
    r.update(passed=True,causality=context.gen43_manual_causality,
        original_atomic_gate_result=context.gen43_manual_rebind_calls,
        states=[dict(state=x.state,message=x.message) for x in context.states],
        observation_motion_count=sum(x[0]=='motion' and x[1]==context.origin.plan_id for x in context.device_calls),
        contact_motion_count=sum(x[0]=='motion' and x[1]==context.contact.plan_id for x in context.device_calls),
        close_count=sum(x[0]=='gripper' and x[2]<.05 for x in context.device_calls),
        frozen_contact_count=len(context.task._frozen_contact_pub.messages),
        diagnostic_limitation='existing station owner check classifies manual takeover as TARGET_LOST/station_task_owner_changed before base MANUAL check; no actual target disappearance inferred')
except BaseException as exc:
    r.update(error=repr(exc),traceback=traceback.format_exc())
    if contexts:
        r.update(causality=contexts[0].gen43_manual_causality,
            original_atomic_gate_result=contexts[0].gen43_manual_rebind_calls,
            states=[dict(state=x.state,message=x.message) for x in contexts[0].states])
finally:
    patch.undo()
    r['sources_unchanged']=all(sha(p)==digest for p,digest in source_sha.items())
    r['passed']=r['passed'] and r['sources_unchanged']
    r['recorded_ns']=time.time_ns()
    (O/'result.json').write_text(json.dumps(r,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (O/'seal.json').write_text(json.dumps({'result_sha256':sha(O/'result.json'),'source_sha256':source_sha},ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(passed=r['passed'],output=str(O),error=r.get('error')),ensure_ascii=False),flush=True)
