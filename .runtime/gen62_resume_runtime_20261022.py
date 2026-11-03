"""Read-only binding of the existing GEN61 runtime after explicit resume69.

Historical source/process evidence remains immutable. The sole changed input,
continuation control, is checked as an explicit 67 -> paused68 -> active69 chain.
No old motion/scene deadline is renewed and no control API is called.
"""
import argparse
import ast
from copy import deepcopy
import hashlib
import importlib.util
import inspect
import json
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
W = Path(__file__).resolve().parents[1]
M = Path('/media/zhuyupei/SanStand')
E = M/'实验记录/Robot-Controller'
N = E/'ros_log/full_latest_gen41_restart_20261018T011222Z'
G = N/'gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1'
RESUME = G/'user_resume_r1'
A = E/'ros_log/full_latest_frozen_no_skin_20261012T021050Z/continuous_authorization_20261012/authorization.json'
CONTROL = A.parent/'control.json'
OLD_CONTROL_SHA = '8beaf2eeabac1d86f217959524083c09b927969f0be4f22646d6c306e6865754'
CONTROL_SHA = '8fe6865ef453dad1808faf32f01da1b5bac0493755a4e934fdb63bc76114d995'
REQUEST = RESUME/'user_resume_request.json'
REQUEST_SHA = '87ed176de63a7612d41d9b1a314421a2033b867ad76f8a05cc84a7c1421b3f15'
RELEASE = N/'verification/gen61_release_manifest.json'
RELEASE_SHA = 'f233ba3a41dca9632a13e084b24c5a0c434c70ba0c28f38aeba369ecfb3c17e9'
ACCEPTED = G/'runtime_reload_r1/acceptance/result.json'
ACCEPTED_SHA = '472b99ebf4c6b3a8665a76f5c417e2748a3d09186ab84599ea46a399954c7a86'
RETURNED = G/'actual_attempt_r1/scene_start_r9/recorded_entry_result.json'
RETURNED_SHA = '261183043edd56a0295816f5ba62c42f2241fb824546460ea11f887a0bc8502f'
R = E/'grasp_records/20261022T092802.096230Z_567d85bbe8f54a56af71145548b078f8'
EPOCH = 1792661888995982113
SELECTION = dict(mode='unknown',strategy='observe_return',generation=5,stamp_ns=1792661288399203300)
TARGETS = ('/grasp_task_node','/remote_grasp6d_node','/unknown_tabletop_perception')


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_bytes())


def load(path, digest, name):
    require(sha(path)==digest,'helper bytes changed: '+str(path))
    spec=importlib.util.spec_from_file_location(name,path)
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod


def save(path,value):
    path=Path(path).resolve()
    require(M.is_mount() and G in path.parents and path.parent.stat().st_dev==M.stat().st_dev
        and not os.statvfs(M).f_flag & os.ST_RDONLY,'external output unavailable')
    raw=(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    with path.open('xb') as f:f.write(raw);f.flush();os.fsync(f.fileno())
    require(path.read_bytes()==raw,'receipt readback differs')


def recheck(ctx,sources=None):
    require(all(sha(p)==h for p,h in ctx['pins'].items()),'current or historical input changed')
    ctl,req=read(CONTROL),read(REQUEST)
    require(ctl['state']=='active' and type(ctl['revision']) is int and ctl['revision']==69
        and ctl['authorization_id']==read(A)['authorization_id']
        and ctl['resume_instruction']==dict(path=str(REQUEST),sha256=REQUEST_SHA)
        and ctl['resume_user_message']=='继续处理'
        and req['source']=='actual_user_message' and req['text']=='继续处理'
        and req['new_hands_clear_confirmation_claimed'] is False,'resume69 binding changed')
    if sources is not None:ctx['v'].source_check(sources)
    require(read(E/'grasp_records/archive_config.json')['upload_enabled'] is False
        and read(E/'grasp_records/archive_config.json')['delete_enabled'] is False
        and read(E/'maintenance/storage_control.json')['archive_policy']=='local_only','storage policy changed')
    require(M.is_mount() and os.statvfs(E).f_bavail*os.statvfs(E).f_frsize >=
        read(E/'grasp_records/archive_config.json')['min_free_bytes'],'external reserve unavailable')
    return ctl


def context():
    require(sha(RETURNED)==RETURNED_SHA and sha(RELEASE)==RELEASE_SHA
        and sha(ACCEPTED)==ACCEPTED_SHA,'accepted release/returned attempt changed')
    returned=read(RETURNED)
    require(returned['original_runner_returned'] is True and returned['gate_returned_to_original_mode_line'] is True
        and returned['record_dir']==str(R) and returned['automatic_retry'] is False,'prior attempt has not returned')
    prior={Path(p):h for p,h in returned['input_sha256'].items()}
    require(len(prior)==414 and prior[CONTROL]==OLD_CONTROL_SHA,'historical input closure differs')
    archive=G/'user_pause_r2/control_before.json'
    require(sha(archive)==OLD_CONTROL_SHA,'original active67 control not preserved exactly')
    # Only this known mutable control path is replaced. Every other historical
    # input remains at its original location with its original digest.
    pins={p:h for p,h in prior.items() if p!=CONTROL}
    pins.update({archive:OLD_CONTROL_SHA,CONTROL:CONTROL_SHA,REQUEST:REQUEST_SHA,
        RETURNED:RETURNED_SHA,RELEASE:RELEASE_SHA,ACCEPTED:ACCEPTED_SHA,
        RESUME/'control_before.json':'6189b0b9b69ed27dba5e8261bea82a39469299c838d8ce6ec81c68012607b031',
        RESUME/'control_after.json':CONTROL_SHA,
        G/'user_pause_r2/current_idle.json':'a03176ebcdfaafc52b68469cfcf49833a52ba08e58e2ce9d8700af77855af542',
        R/'authorization.json':'d5fb4756d227d453c57cb5c13f0e2667058afd1ae03b219ce91f5963036ea5ec',
        R/'user_request_runner_exit.json':'e0446cb32bb22c77ff3618cbf430a16db32fdbb06c75248634ee546612fe2148'})
    require(all(sha(p)==h for p,h in pins.items()),'frozen resume inputs differ')
    paused,active=read(RESUME/'control_before.json'),read(CONTROL)
    require(paused['state']=='paused' and paused['revision']==68 and active['revision']==paused['revision']+1
        and paused['authorization_id']==active['authorization_id']==read(archive)['authorization_id'],
        'explicit pause/resume succession differs')
    marker=read(G/'user_pause_r2/current_idle.json')
    require(marker['passed'] is True and marker['current_epoch_ns']==EPOCH
        and marker['manual_mode'] is True and not marker['task_active'] and not marker['actions_active'],
        'last observed reference is not a qualified starting expectation')
    def checked(relative,name):
        path=W/'.runtime'/relative
        require(path in pins,'helper missing from accepted input closure')
        return load(path,pins[path],name)
    base=checked('gen60_gateway_reload_common_20261022.py','gen62_unmodified_base60')
    legacy=checked('gen59_remote_reload_common_20261022.py','gen62_unmodified_identity59')
    old=checked('gen58_cold_attempt_common_20261021.py','gen62_original_publishers58')
    verifier=checked('verify_gen58_full_cold_restart_20261021.py','gen62_original_process_verifier58')
    runtime=checked('gen61_compute_runtime_20261022.py','gen62_unmodified_graph61')
    checker=checked('verify_active_remote_readiness_20261020.py','gen62_original_dependency_check')
    config_path=N/'gen58_resume_r1/full_restart_r1/startup_configuration.json'
    startup_path=N/'gen58_resume_r1/full_restart_r1/startup.json'
    require(config_path in pins and startup_path in pins,'original complete startup not bound')
    release,accepted=read(RELEASE),read(ACCEPTED)
    bundle,transition=read(accepted['bundle']),read(accepted['reload'])
    validator=checked('gen61_compute_attempt_common_20261022.py','gen62_prior_acceptance_validator')
    nodes=validator.validate_acceptance_values(release,accepted,bundle,transition)
    require(release['acceptance']==dict(path=str(ACCEPTED),sha256=ACCEPTED_SHA)
        and all(sha(p)==h for p,h in release['evidence_sha256'].items()),'GEN61 release evidence changed')
    # Reuse historical pure validators; do not call their old active67 context.
    previous=read(runtime.ACCEPTED)
    startup=read(startup_path)
    require(startup['boot_id']==Path('/proc/sys/kernel/random/boot_id').read_text().strip(),'boot changed')
    require(base.selection_identity(read(R/'authorization.json')['selection'])==SELECTION
        and base.selection_identity(read(R/'user_request_runner_exit.json')['selection'])==SELECTION,
        'returned current selection differs')
    ctx=dict(old=old,v=verifier,runtime59=legacy,checker=checker,pins=pins,
        previous=previous,config=read(config_path),startup=startup,nodes=nodes,
        preserved={n:r for n,r in nodes.items() if n not in TARGETS},
        expected_gateway=nodes['/motion_gateway'],release=release,accepted=accepted,
        bundle=bundle,transition=transition,base=base,runtime=runtime)
    binding=transition['remote_binding'];require(sha(binding['path'])==binding['sha256'],'remote binding changed')
    ctx['binding']=checker.load_binding(Path(binding['path']),binding['sha256'])
    require(ctx['binding']['parameters']==ctx['config']['remote_parameters']
        and all(ctx['binding']['node_identity'][k]==nodes['/remote_grasp6d_node'][k]
            for k in ('pid','parent_pid','start_ticks')),'current remote instance/config differs')
    recheck(ctx,release['source_sha256'])
    return ctx


def idle_function(ctx):
    base=ctx['base'];tree=ast.parse(inspect.getsource(base.sample_idle));count=[0]
    old=ast.dump(ast.parse("selection['generation']==3",mode='eval').body,include_attributes=False)
    class Adapt(ast.NodeTransformer):
        def visit_Compare(self,node):
            if ast.dump(node,include_attributes=False)==old:
                count[0]+=1;node.comparators[0]=ast.Constant(SELECTION['generation'])
            return self.generic_visit(node)
    tree=Adapt().visit(tree);ast.fix_missing_locations(tree)
    require(count==[1],'original sampling contract shape differs')
    ns=dict(base.__dict__,SELECTION=SELECTION,recheck=recheck)
    exec(compile(tree,str(W/'.runtime/gen60_gateway_reload_common_20261022.py')+'#GEN62_current_reference','exec'),ns)
    return ns['sample_idle']


def graph(ctx,rospy,master):
    value=ctx['runtime'].graph(ctx['base'],ctx,rospy,master,ctx['accepted']['targets'],
        ctx['transition']['units'],ctx['accepted']['auxiliary_nodes'])
    require(value['node_identities']==ctx['nodes']
        and value['services']==ctx['accepted']['after']['services'],'current runtime ownership differs')
    return value


def snapshot(output):
    ctx=context();out=Path(output).resolve()
    require(RESUME in out.parents and M.is_mount(),'resume output root required');out.mkdir(exist_ok=False)
    os.environ['ROS_LOG_DIR']=str(out/'ros')
    result=dict(kind='GEN62_current_GEN61_resume69_readonly_snapshot',passed=False,
        helper_sha256=sha(__file__),started_ns=time.time_ns(),motion_commands_sent=False,
        mode_changes_sent=False,temperature_queried=False,joint_disable_commands_sent=False,
        scene_clearance_granted=False,automatic_attempt_authorized=False,
        previous_motion_deadlines_refreshed=False)
    try:
        import rosgraph
        import rospy
        rospy.init_node('gen62_resume_readonly',anonymous=True,disable_rosout=True)
        master=rosgraph.Master(rospy.get_name())
        before=graph(ctx,rospy,master)
        sample=idle_function(ctx)(ctx,rospy,master,SELECTION,EPOCH)
        construction=read(ctx['transition']['construction'])
        proof=ctx['runtime'].source_process_proof(ctx['base'],ctx,ctx['bundle'],construction,
            ctx['accepted']['targets'],Path(ctx['accepted']['reload']).parent)
        remote_out=out/'remote_readiness';remote_out.mkdir()
        remote=ctx['checker'].evaluate(ctx['binding'],remote_out);save(remote_out/'result.json',remote)
        require(remote['passed'] and remote['dependency_ready'],'current remote dependency not ready')
        after=graph(ctx,rospy,master);require(before==after,'runtime graph changed during snapshot')
        recheck(ctx,ctx['release']['source_sha256'])
        result.update(sample)
        result.update(passed=True,before=before,after=after,
            source_process_proof=proof,source_sha256=ctx['release']['source_sha256'],
            remote_readiness=dict(path=str(remote_out/'result.json'),sha256=sha(remote_out/'result.json')),
            prior_release=dict(path=str(RELEASE),sha256=RELEASE_SHA),
            prior_deployment=dict(path=str(ACCEPTED),sha256=ACCEPTED_SHA),
            historical_control_archive=dict(path=str(G/'user_pause_r2/control_before.json'),sha256=OLD_CONTROL_SHA),
            current_resume=dict(path=str(REQUEST),sha256=REQUEST_SHA))
    except BaseException as exc:
        import traceback
        result.update(error=repr(exc),traceback=traceback.format_exc())
    result['input_sha256']={str(p):h for p,h in ctx['pins'].items()}
    result['finished_ns']=time.time_ns();save(out/'result.json',result)
    print(json.dumps({k:result.get(k) for k in ('passed','error','epoch_ns','selection','sample_count','actuation_status','manual_mode')},ensure_ascii=False))
    return 0 if result['passed'] else 2


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--helper-sha256',required=True)
    args=parser.parse_args()
    require(sha(__file__)==args.helper_sha256 and not sys.flags.optimize,'exact unoptimized helper required')
    sys.exit(snapshot(args.output))
