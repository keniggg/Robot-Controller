"""One authorized reload/grasp attempt; preserve exact deployment and selection receipts."""
import json
import os
from pathlib import Path
import subprocess
import sys
from alicia_grasp_modes.selection import parse_selection

WORK=Path(__file__).resolve().parents[1]
EXT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN=EXT/'ros_log/full_ros_resume_20261004T002930Z'
ORIGINAL=EXT/'ros_log/rgb_observation_new_trial_20261004T063140.568948Z/result.json'
INTERMEDIATE=EXT/'ros_log/rgb_observation_new_trial_20261004T072512.280310Z/result.json'
SECOND=EXT/'ros_log/rgb_observation_new_trial_20261004T075538.835989Z/result.json'
REFERENCE=EXT/'ros_log/rgb_observation_bootstrap_20261004T061706Z/result.json'
MAINTENANCE=RUN/'driver_temperature_off_20261004T084125Z/result.json'


def exact_selection_result(report, retry_terminal):
    if not report.get('loaded') or not report.get('preserved_nodes_unchanged'):
        raise RuntimeError('Unverified deployment; grasp not started')
    retry=report['prestart_retry']
    if retry.get('returncode') != 0:
        raise RuntimeError('Failed selection transition; grasp not started')
    entries=[]
    for line in retry_terminal.splitlines():
        try: item=json.loads(line)
        except ValueError: continue
        if (isinstance(item,dict) and item.get('new_selection') is not None
                and parse_selection(item['new_selection'])==parse_selection(retry['selection'])):
            entries.append(item)
    if len(entries)!=1 or entries[0].get('preview_only') is not False or entries[0].get('failure'):
        raise RuntimeError('Missing exact current selection transition; grasp not started')
    result=Path(entries[0]['path'])/'result.json'
    if EXT not in result.resolve().parents:
        raise RuntimeError('Non-archive result refused')
    return result


def main():
    out=Path(os.environ['GRASP_RECORD_DIR']).resolve()
    if EXT not in out.parents or not (out/'recorder_ready.json').is_file():
        raise RuntimeError('Active external full recording required')
    cmd=['/usr/bin/python3',str(WORK/'.runtime/load_rgb_acquisition_20261004.py'),
         '--perception-only','--carry-forward-result',str(ORIGINAL),
         '--unreserved-transition',str(INTERMEDIATE),
         '--unreserved-transition',str(SECOND),
         '--maintenance-receipt',str(MAINTENANCE),
         '--aligned-observation-record',str(REFERENCE),
         '--aligned-source-stamp-ns','1791094630503762245']
    run=subprocess.run(cmd,cwd=str(WORK),stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    print(run.stdout,flush=True)
    (out/'perception_reload.log').write_text(run.stdout)
    if run.returncode:
        return run.returncode
    # Loader stdout ends with its JSON report; systemd status may precede it.
    offset=run.stdout.find('{\n  "started_at_utc"')
    if offset<0:
        raise RuntimeError('Missing exact loader receipt; grasp not started')
    report=json.loads(run.stdout[offset:])
    retry=report['prestart_retry']
    result=exact_selection_result(report,Path(retry['terminal_log']).read_text())
    return subprocess.call(['/usr/bin/python3',str(WORK/'.runtime/run_confirmed_unknown_grasp_20261004.py'),
        '--selection-result',str(result)],cwd=str(WORK))

if __name__=='__main__':
    sys.exit(main())
