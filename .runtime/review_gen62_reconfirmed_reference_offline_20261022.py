"""Pure review probes only; no reference snapshot or original context is executed."""
import ast
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

W = Path(__file__).resolve().parents[1]
G = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1')
F = G / 'contact_feedback_fix_r1'
HELPER = W / '.runtime/gen62_reconfirmed_reference_20261022.py'
EXPECTED = 'f7c4eecb6d7ed26b400e0c13957e4394ec4cb8fb2312a21b4af1ce8ccb6e2928'
ORIGINAL = W / '.runtime/gen62_resume_runtime_20261022.py'
ORIGINAL_SHA = 'd9f1ac014f09b5743a7bddedaa8219b386161d41bd5c3eacebf30afcd0ee3127'
PROOF = F / 'manual_reference_evidence_r1/result.json'
REVIEW = F / 'reconfirmed_reference_review_r1/result.json'
OBSERVATION = F / 'manual_reconfirmation_observer_r1'
EPOCH = 1792668378941525117
OLD_EPOCH = 1792661888995982113
REPLY_SHA = '65c7b4ff9c905d00c5e6306d8b87a43c02170dc90ba93c66389138744d6bda7a'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def main():
    assert not sys.flags.optimize
    assert digest(HELPER) == EXPECTED and digest(ORIGINAL) == ORIGINAL_SHA
    boundary_attempts = []

    def audit(event, args):
        if event.startswith(('socket.', 'subprocess.', 'os.exec', 'os.fork', 'ctypes.')):
            boundary_attempts.append(event)
            raise RuntimeError('offline boundary: ' + event)

    sys.addaudithook(audit)
    tree = ast.parse(HELPER.read_text())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'current_context')
    observer = W / '.runtime/observe_gen62_manual_reconfirmation_20261022.py'
    historical = F / 'synthetic_historical_input_not_opened.json'
    hashes = {PROOF:'a'*64, REVIEW:'b'*64, ORIGINAL:ORIGINAL_SHA, HELPER:EXPECTED,
              OBSERVATION/'events.jsonl':'c'*64, OBSERVATION/'result.json':'d'*64,
              OBSERVATION/'user_reply_actual_motion.json':REPLY_SHA, observer:'e'*64,
              historical:'f'*64}
    proof = dict(passed=True, before_epoch_ns=OLD_EPOCH, after_epoch_ns=EPOCH,
                 gui_positive_enable_count=1, negative_enable_count=0, gui_command_count=17,
                 actual_sdk_motion_observed=True, confirmed_observed=True, observer_process_closed=True,
                 driver_callerid='/alicia_d_driver_node', gui_callerid='/alicia_supervisor_gui',
                 input_sha256={str(p):hashes[p] for p in (OBSERVATION/'events.jsonl',
                     OBSERVATION/'result.json',OBSERVATION/'user_reply_actual_motion.json',observer)})
    fixture = {'proof':proof,'review':dict(passed=True,helper_sha256=EXPECTED),
               'reply':dict(source='actual_user_message',text='已观察到机械臂实际移动，并离开桌边'),
               'hashes':hashes,'prior_pins':{historical:hashes[historical]},'historical_reject':False}

    def run(change=None):
        value=copy.deepcopy(fixture)
        if change is not None:
            change(value)
        calls=[]

        def original_context():
            calls.append('original_context')
            if value['historical_reject']:
                raise RuntimeError('historical context refused')
            return dict(pins=dict(value['prior_pins']),release={'source_sha256':{'source':'source-sha'}})

        def read(path):
            return {PROOF:value['proof'],REVIEW:value['review'],
                    OBSERVATION/'user_reply_actual_motion.json':value['reply']}[path]

        def recheck(ctx,sources):
            calls.append('original_recheck')
            require(all(value['hashes'].get(p)==h for p,h in ctx['pins'].items()),'recheck changed pins')
            require(sources=={'source':'source-sha'},'source map replaced')

        original=SimpleNamespace(context=original_context,EPOCH=OLD_EPOCH,read=read,recheck=recheck)
        ns=dict(Path=Path,F=F,G=G,W=W,PROOF=PROOF,REVIEW=REVIEW,OBSERVATION=OBSERVATION,
                ORIGINAL=ORIGINAL,ORIGINAL_SHA=ORIGINAL_SHA,EPOCH=EPOCH,__file__=str(HELPER),
                require=require,sha=lambda p:value['hashes'].get(Path(p)))
        exec(compile(ast.Module(body=[function],type_ignores=[]),str(HELPER)+'#context-pure-probe','exec'),ns)
        try:
            ctx=ns['current_context'](original,'a'*64,'b'*64)
        except RuntimeError as exc:
            return False,str(exc),calls
        assert original.EPOCH==OLD_EPOCH and calls==['original_context','original_recheck']
        assert ctx['pins'][HELPER]==EXPECTED and ctx['pins'][PROOF]=='a'*64 and ctx['pins'][REVIEW]=='b'*64
        return True,None,calls

    assert run()[0]
    cases=[]
    for key,value in [('passed',False),('before_epoch_ns',OLD_EPOCH+1),('after_epoch_ns',EPOCH+1),
                      ('gui_positive_enable_count',0),('negative_enable_count',1),('gui_command_count',16),
                      ('actual_sdk_motion_observed',False),('confirmed_observed',False),
                      ('observer_process_closed',False),('driver_callerid','/foreign'),('gui_callerid','/foreign')]:
        cases.append((key,lambda state,k=key,v=value:state['proof'].__setitem__(k,v)))
    cases.extend([
        ('historical_context_reject',lambda s:s.__setitem__('historical_reject',True)),
        ('proof_hash_change',lambda s:s['hashes'].__setitem__(PROOF,'0'*64)),
        ('review_hash_change',lambda s:s['hashes'].__setitem__(REVIEW,'0'*64)),
        ('review_failed',lambda s:s['review'].__setitem__('passed',False)),
        ('review_wrong_helper',lambda s:s['review'].__setitem__('helper_sha256','0'*64)),
        ('missing_events_pin',lambda s:s['proof']['input_sha256'].pop(str(OBSERVATION/'events.jsonl'))),
        ('input_bytes_changed',lambda s:s['hashes'].__setitem__(OBSERVATION/'events.jsonl','0'*64)),
        ('reply_hash_changed',lambda s:(s['proof']['input_sha256'].__setitem__(str(OBSERVATION/'user_reply_actual_motion.json'),'0'*64),s['hashes'].__setitem__(OBSERVATION/'user_reply_actual_motion.json','0'*64))),
        ('reply_text_changed',lambda s:s['reply'].__setitem__('text','different')),
        ('reply_not_actual',lambda s:s['reply'].__setitem__('source','assumed')),
        ('foreign_input_path',lambda s:s['proof']['input_sha256'].__setitem__('/tmp/unread_manual_proof','0'*64)),
        ('prior_pin_conflict',lambda s:s['prior_pins'].__setitem__(OBSERVATION/'events.jsonl','0'*64)),
    ])
    results=[]
    for name,change in cases:
        passed,reason,calls=run(change)
        assert not passed,name
        results.append(dict(case=name,rejected=True,reason=reason,calls=calls))

    # Load definitions only. Original.context/snapshot remain uncalled.
    spec=importlib.util.spec_from_file_location('gen62_review_historical_definitions',ORIGINAL)
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    import inspect
    snapshot_source=inspect.getsource(old.snapshot)
    namespace=dict(old.__dict__,EPOCH=EPOCH,__file__=str(HELPER),context=lambda:None)
    exec(compile(ast.parse(snapshot_source),str(ORIGINAL)+'#independent-namespace-proof','exec'),namespace)
    assert namespace['snapshot'].__globals__['EPOCH']==EPOCH
    assert old.context.__globals__['EPOCH']==OLD_EPOCH and old.EPOCH==OLD_EPOCH
    assert namespace['idle_function'] is old.idle_function and namespace['graph'] is old.graph
    assert namespace['recheck'] is old.recheck and old.SELECTION['generation']==5
    assert boundary_attempts==[]
    result=dict(kind='GEN62_reconfirmed_reference_independent_pure_contract',passed=True,
                helper_sha256=EXPECTED,original_sha256=ORIGINAL_SHA,
                positive=1,negative=len(results),negative_cases=results,
                snapshot_ast_sha256=hashlib.sha256(ast.dump(ast.parse(snapshot_source),include_attributes=False).encode()).hexdigest(),
                original_context_globals_unchanged=True,current_snapshot_namespace_epoch=EPOCH,
                original_context_namespace_epoch=OLD_EPOCH,original_graph_idle_recheck_function_objects_retained=True,
                scope='Synthetic proof/receipt namespace for exact current_context AST; original definitions loaded but no original context, snapshot, ROS, graph, PID or hardware work executed.',
                boundary_attempts=boundary_attempts,actual_manual_proof_validated=False,actual_readiness_sampled=False)
    out=F/'reconfirmed_reference_review_r1';out.mkdir(exist_ok=True)
    path=out/'offline_contract.json';raw=(json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode()
    with path.open('xb') as stream:
        stream.write(raw);stream.flush();os.fsync(stream.fileno())
    assert path.read_bytes()==raw
    print(json.dumps(dict(path=str(path),sha256=digest(path),positive=1,negative=len(results)),ensure_ascii=False))


if __name__=='__main__':
    main()
