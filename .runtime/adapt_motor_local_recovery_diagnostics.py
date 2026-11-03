#!/usr/bin/env python3
"""Import archived motor diagnostic evidence without target or motion promotion."""
import argparse,hashlib,importlib.util,json,os
from pathlib import Path
HERE=Path(__file__).resolve().parent
s=importlib.util.spec_from_file_location('local_evidence',HERE/'local_recovery_evidence.py');L=importlib.util.module_from_spec(s);s.loader.exec_module(L)
MOUNT=Path('/media/zhuyupei/SanStand')
def run(source,output):
    rows=[];failures={}
    for p in sorted(source.glob('*/report_with_provenance.json')):
        blob=p.read_bytes();report=json.loads(blob);sha=hashlib.sha256(blob).hexdigest()
        for frame in report['frames']:
            key=str(p)+':'+str(frame['source_stamp_ns'])
            try:
                evidence=L.LocalEvidence.from_motor_diagnostic(report,frame,sha)
                rows.append(dict(source_report=str(p),source_report_sha256=sha,evidence_digest=evidence.evidence_digest,level=evidence.level,payload=evidence.data,motion_authorized=False))
            except (ValueError,KeyError,TypeError) as exc:failures[key]=str(exc)
    result=dict(schema=1,kind='source_preserving_local_diagnostic_adapter',evidence_count=len(rows),evidence=rows,rejected=failures,contexts_created=0,production_target_bindings_created=0,motion_authorized=False,reason='Manual motor ROI is not a production track; no historical/current identity or current source time is invented.')
    output.mkdir(parents=True,exist_ok=True)
    (output/'adapted_motor_diagnostics.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    for file in ('local_recovery_evidence.py','adapt_motor_local_recovery_diagnostics.py','test_local_recovery_evidence.py'):(output/file).write_bytes((HERE/file).read_bytes())
    print(json.dumps(dict(evidence_count=len(rows),rejected=failures,contexts_created=0,motion_authorized=False),ensure_ascii=False))
if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if not os.path.ismount(MOUNT) or MOUNT not in a.output.resolve().parents:raise RuntimeError('Mounted SanStand output required')
    run(a.source,a.output)
