"""Read-only profile trace for the original GEN61 interleaved failure case."""
import json
import os
from pathlib import Path
import sys
import traceback

_ROWS=[]
_WATCH={'_station_compute_retire','_station_compute_quarantine','_station_private_pending_input','_station_compute_join_locked','_station_private_pending_reconcile','wire'}

def state(node):
    ref=getattr(node,'_station_compute_reference',None)
    rows=getattr(node,'_station_private_pending_sources',{})
    status=getattr(node,'_station_compute_status',{})
    return dict(cutoff_ns=getattr(node,'_station_compute_cutoff_ns',None),
        soft_cutoff_ns=getattr(node,'_station_compute_soft_cutoff_ns',None),
        reference_source_ns=None if ref is None else ref['contract']['source_ns'],
        pending=getattr(node,'_station_compute_pending',None),retired=getattr(node,'_station_compute_retired',None),
        private_rows={str(k):dict(kind=v.get('kind'),seed_source_ns=v.get('seed_receipt',{}).get('source_ns'),
            receipt_parent_ns=v.get('seed_receipt',{}).get('parent_source_ns')) for k,v in rows.items()},
        cached_statuses={str(k):dict(state=v.get('state'),seed_receipt=v.get('station_seed_insufficiency'),
            recovery_receipt=v.get('station_seed_insufficiency_recovery'),parent_ns=(v.get('station_compute_recovery') or {}).get('source_ns')) for k,v in status.items()})

def profile(frame,event,arg):
    name=frame.f_code.co_name
    if event not in ('call','return') or name not in _WATCH:
        return
    local=frame.f_locals
    node=local.get('self')
    if name=='wire':
        context=local.get('c')
        node=getattr(context,'node',None)
    if node is None or not hasattr(node,'_station_compute_reference'):
        return
    row=dict(event=event,function=name,line=frame.f_lineno,state=state(node))
    for key in ('kind','ns','reason','source_ns','clock_only','preserve_unpublished_work'):
        if key in local and isinstance(local[key],(str,int,bool,type(None))):row[key]=local[key]
    if name=='wire':
        pub=local.get('row',{})
        s=pub.get('status',{})
        row['input']=dict(order=list(local.get('order',('status','object','mask'))),
            source_ns=s.get('source_stamp_ns'),state=s.get('state'),
            recovery_receipt=s.get('station_seed_insufficiency_recovery'))
    if name=='_station_compute_retire' and event=='call':
        row['stack']=traceback.format_stack(frame)
    _ROWS.append(row)

def pytest_runtest_setup(item):
    sys.setprofile(profile)

def pytest_runtest_teardown(item):
    sys.setprofile(None)
    path=Path(os.environ['GEN61_DIAGNOSTIC_TRACE'])
    with path.open('x') as stream:
        json.dump(_ROWS,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
