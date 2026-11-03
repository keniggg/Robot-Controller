#!/usr/bin/env python3
"""Source-pinned, no-I/O FK reproduction of the four sealed GEN59 rejects."""
import argparse
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
sys.dont_write_bytecode=True
from replay_gen59_held_axes_20261022 import N, W, CONTEXT, RELEASE, sha


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--source-root',type=Path,required=True)
    p.add_argument('--expected-selector-sha256',required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();c=args.source_root.resolve();out=args.output.resolve()
    assert str(out).startswith(str(N)+'/') and Path('/media/zhuyupei/SanStand').is_mount()
    out.mkdir(parents=True,exist_ok=False)
    platform._uname_cache=platform.uname_result(*os.uname(),'')
    blocked=[]
    def guard(event,values):
        deny=event.startswith('socket.') or event in ('subprocess.Popen','os.system','os.fork','os.posix_spawn','os.exec')
        if event=='open' and isinstance(values[0],(str,bytes,os.PathLike)):
            f=Path(values[0]).resolve();m,flags=values[1:3]
            write=(isinstance(m,str) and any(v in m for v in 'wax+')) or (isinstance(flags,int) and bool(flags&(os.O_WRONLY|os.O_RDWR|os.O_CREAT|os.O_TRUNC|os.O_APPEND)))
            deny|=write and out not in f.parents
            deny|=str(f).startswith('/dev/') and str(f)!='/dev/null'
        if deny:blocked.append(dict(event=event,args=str(values)[:200]));raise RuntimeError('offline denied '+event)
    sys.addaudithook(guard)
    rel='src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/held_pregrasp_selection.py'
    assert sha(c/rel)==args.expected_selector_sha256
    sys.path[:0]=[str(c/'src/alicia_flexible_grasp_supervisor/src'),'/opt/ros/noetic/lib/python3/dist-packages','/usr/lib/python3/dist-packages']
    from alicia_flexible_grasp.robot.held_pregrasp_selection import select_held_pregrasp_endpoint
    from alicia_flexible_grasp.robot.held_axes_prediction import HeldAxesPredictionError
    from alicia_flexible_grasp.robot.stationary_following import SerialUrdfFk,ARM_NAMES
    pins=json.loads(RELEASE.read_text())['source_sha256'];pins[rel]=args.expected_selector_sha256
    sources={}
    for module in list(sys.modules.values()):
        f=getattr(module,'__file__',None)
        if f and str(f).startswith(str(c)+'/'):
            k=str(Path(f).relative_to(c));sources[k]=sha(f);assert sources[k]==pins[k]
    fixture=N/'gen58_resume_r1/observation_pose_fix/actual_attempt_r1/held_axes_review_r1/minimal_fixture.json'
    baseline=N/'gen58_resume_r1/observation_pose_fix/actual_attempt_r1/held_axes_review_r1/replay_r2/result.json'
    inputs={str(f):sha(f) for f in (fixture,baseline,CONTEXT,RELEASE)}
    data=json.loads(fixture.read_text());old=json.loads(baseline.read_text())
    fk=SerialUrdfFk(json.loads(CONTEXT.read_text())['/robot_description'],ARM_NAMES)
    assert fk.model_sha256==data['model_sha256']
    rows=[];start=time.monotonic()
    for case,base in zip(data['rows'],old['rows']):
        tick=time.monotonic();calls=[0]
        def count_check():calls[0]+=1
        try:
            command,report=select_held_pregrasp_endpoint(fk,case['sample'],case['pregrasp_goal'],case['approach_goal'],case['nominal_command_rad'],position_tolerance_m=case['position_tolerance_m'],orientation_tolerance_rad=case['orientation_tolerance_rad'],deadline_check=count_check)
            row=dict(success=True,command_rad=command.tolist(),report=report)
        except HeldAxesPredictionError as exc:
            report=exc.evidence['pregrasp_endpoint_selection'];row=dict(success=False,code=str(exc),report=report)
        original=base['recorded']['held_pregrasp_selection']
        row.update(lineage_index=case['lineage_index'],plan_id=case['plan_id'],elapsed_sec=time.monotonic()-tick,check_callback_count=calls[0],original_trial_commands_exact=[a['quantized_command_rad']==b['quantized_command_rad'] for a,b in zip(original['trials'],report['trials'])],original_trial_codes_exact=[a.get('rejection')==b.get('rejection') for a,b in zip(original['trials'],report['trials'])])
        rows.append(row)
        print(json.dumps(dict(lineage_index=case['lineage_index'],success=row['success'],elapsed_sec=row['elapsed_sec'],lattice_sec=report['quantized_neighborhood']['elapsed_sec'],checks=len(report['quantized_neighborhood']['checked']))),flush=True)
    result=dict(kind='gen60_recorded_held_lattice_candidate_focus',source_root=str(c),source_sha256=sources,inputs=inputs,model_sha256=fk.model_sha256,rows=rows,elapsed_sec=time.monotonic()-start,blocked_events=blocked,execution_authorized=False,limits=['Pure conditional FK endpoint prediction only. Check callback counts are instrumentation, not proof of historical real deadline or feedback freshness.','No ROS node, network, motion, trajectory, collision or hardware-response verification.'])
    result['passed']=([r['success'] for r in rows]==[True,True,False,True] and all(all(r['original_trial_commands_exact']) and all(r['original_trial_codes_exact']) for r in rows))
    for key,digest in sources.items():assert sha(c/key)==digest
    for key,digest in inputs.items():assert sha(key)==digest
    (out/'result.json').write_text(json.dumps(result,indent=2,sort_keys=True,allow_nan=False)+'\n')
    print(json.dumps(dict(result=str(out/'result.json'),sha256=sha(out/'result.json'),passed=result['passed'])),flush=True)


if __name__=='__main__':main()
