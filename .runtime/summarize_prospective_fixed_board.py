#!/usr/bin/env python3
"""Aggregate immutable selected-frame reports; never reacquire, refit or clear faults."""
import argparse,hashlib,json,os
from pathlib import Path
import evaluate_prospective_metric_candidate as E


def aggregate(directories):
    baseline=None;rows=[];audits={};seen=set();sources=[];layout=None
    for directory in directories:
        report_path=directory/'report.json';frames_path=directory/'selected_frames.json'
        report=json.loads(report_path.read_text());frames=json.loads(frames_path.read_text())
        expected={'candidate_model_sha256':E.FROZEN_MODEL,'candidate_handeye_sha256':E.FROZEN_X,
                  'frozen_fit_sha256':E.FROZEN_FIT,'evaluator_source_sha256':E.sha(E.__file__)}
        if any(report.get(k)!=v for k,v in expected.items()):raise ValueError('aggregate requires exact frozen evaluator and candidate identities')
        if layout is None:layout=report['board_layout_id']
        if layout!=report['board_layout_id']:raise ValueError('fixed board layouts differ')
        if baseline is None:baseline=frames['baseline']
        if baseline!=frames['baseline']:raise ValueError('baseline raw frames or frozen transformed Y inputs differ')
        for row in frames['validation']:
            key=(row['session'],row['source_stamp_ns'])
            if key in seen:raise ValueError('duplicate validation frame across input reports')
            seen.add(key);rows.append(row)
        for audit in report['session_audits']:
            key=audit['session']
            if key in audits and audits[key]!=audit:raise ValueError('session audit evidence differs')
            audits[key]=audit
        sources.append(dict(report=str(report_path),report_sha256=E.sha(report_path),frames=str(frames_path),frames_sha256=E.sha(frames_path)))
    result=E.add_coverage(E.evaluate(baseline,rows),list(audits.values()))
    outcomes=[]
    for session in audits:
        path=Path(session)/'terminal.json'
        if path.exists():
            terminal=json.loads(path.read_text())
            outcomes.append(dict(session=session,terminal=terminal,sha256=E.sha(path),
                failed_executed_command_contract=bool(terminal.get('commands_published',0)>0 and terminal.get('status')=='failed')))
        else:outcomes.append(dict(session=session,terminal_available=False,failed_executed_command_contract=None))
    failed=[o['session'] for o in outcomes if o.get('failed_executed_command_contract') is True]
    result.update(schema=1,board_layout_id=layout,diagnostic_only=True,fit_performed=False,ros_parameters_written=False,motor_commands_sent=0,
        candidate_model_sha256=E.FROZEN_MODEL,candidate_handeye_sha256=E.FROZEN_X,frozen_fit_sha256=E.FROZEN_FIT,
        evaluator_source_sha256=E.sha(E.__file__),aggregate_source_sha256=E.sha(__file__),input_reports=sources,
        session_audits=list(audits.values()),control_session_outcomes=outcomes,failed_executed_command_contracts=failed,
        control_contract_outcome='FAILED_PREVIOUS_COMMAND_CONTRACT_NOT_CLEARED' if failed else 'READ_ORIGINAL_SESSION_OUTCOMES',
        interpretation='A later zero-command stationary observation is a separate phase/session. It does not retroactively supply a missing original command hold window or clear an earlier actuation envelope fault. Numeric pose thresholds and motion-contract outcomes are independent facts.')
    return result,dict(baseline=baseline,validation=rows)


def main():
    p=argparse.ArgumentParser();p.add_argument('reports',nargs='+',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not os.path.ismount(E.MOUNT) or E.MOUNT not in a.output.resolve().parents:raise ValueError('SanStand output required; no fallback')
    if a.output.exists():raise ValueError('refusing report overwrite')
    E.validate_model_pins(json.loads(E.DEFAULT_MANIFEST.read_text()))
    result,frames=aggregate(a.reports);a.output.mkdir(parents=True)
    (a.output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (a.output/'selected_frames.json').write_text(json.dumps(frames,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    (a.output/'aggregate_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(dict(output=str(a.output),status=result['status'],control_contract_outcome=result['control_contract_outcome'],validation_frame_count=len(frames['validation']),
        metrics={n:m['all_validation'] for n,m in result['models'].items()},phases=result['coverage']['command_phases']),ensure_ascii=False))

if __name__=='__main__':main()
