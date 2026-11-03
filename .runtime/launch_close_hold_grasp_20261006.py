"""Launch one recorded grasp from the current explicit alignment selection."""
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess

W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'

def main():
    assert M.is_mount(), 'SanStand required; no fallback'
    index = json.loads((W/'.runtime/current_alignment_close_hold_20261006.json').read_text())
    selected = Path(index['receipt']).parent/'new_target_selection.json'
    selection = json.loads(selected.read_text())
    assert selection['response']['success'] and not selection.get('failure')
    assert selection['preview_only'] is False
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out = E/'ros_log'/('grasp_close_hold_'+stamp)
    out.mkdir(exist_ok=False)
    (out/'ros').mkdir()
    unit = 'alicia-grasp-close-hold-'+stamp.lower()
    command = ['systemd-run','--user','--collect','--unit='+unit,
        '--property=Type=exec','--property=Restart=no',
        '--property=WorkingDirectory='+str(W),
        '--property=StandardOutput=append:'+str(out/'launcher.log'),
        '--property=StandardError=append:'+str(out/'launcher.log'),
        '--setenv=ROS_LOG_DIR='+str(out/'ros'),
        '/bin/bash',str(W/'.runtime/start_recorded_grasp_20261006.sh'),str(selected)]
    report = dict(directory=str(out),unit=unit,argv=command,
        selection=selection['new_selection'],alignment_receipt=index['receipt'],
        user_request='已换好新目标并对准，可以抓取',physical_grip_verified=False)
    result = subprocess.run(command,text=True,capture_output=True,timeout=15)
    report.update(returncode=result.returncode,output=result.stdout+result.stderr)
    (out/'launch.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (W/'.runtime/current_grasp_close_hold_20261006.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2),flush=True)
    result.check_returncode()

if __name__=='__main__':
    main()
