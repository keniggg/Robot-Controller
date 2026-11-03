"""Restore one accepted compute environment and run its original entry once.

This bootstrap does not select a mode or issue a motion command. Its receipt
proves source/cache/startup inputs; it is not loaded-memory or ROS attestation.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import runpy
import sys
import time


def require(ok, reason):
    if not ok:
        raise RuntimeError(reason)


def verified_json(path, digest):
    raw=Path(path).read_bytes()
    require(hashlib.sha256(raw).hexdigest()==digest,'startup input changed: '+str(path))
    return json.loads(raw)


def main():
    require(not sys.flags.optimize,'unoptimized Python required')
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--construction',type=Path,required=True)
    parser.add_argument('--construction-sha256',required=True)
    parser.add_argument('--node',required=True)
    parser.add_argument('--environment-restored',action='store_true')
    args,ros_args=parser.parse_known_args()
    construction=verified_json(args.construction,args.construction_sha256)
    require(construction['kind']=='three_compute_owner_construction','generic construction required')
    process=construction['processes'][args.node]
    require(ros_args==process['ros_args'],'original ROS argument/remapping changed')
    bundle=verified_json(construction['bundle'],construction['bundle_sha256'])
    require(bundle['deployment_ready'] is True,'reviewed update bundle required')
    from compute_owner_update import UpdateTransaction, rebind, sha, P
    with UpdateTransaction(bundle['contract'],bundle['contract_sha256']) as session:
        require(session.data['starter']==dict(path=str(Path(__file__).resolve()),sha256=sha(__file__))
            and bundle['source_sha256']==session.sources
            and construction['source_sha256']==session.sources
            and construction['epoch_ns']==session.expected_epoch
            and construction['selection']==session.expected_selection,'startup contract/source/reference differs')
        session.phase_sources=session.sources
        session.recheck()
        if not args.environment_restored:
            os.chdir(process['cwd'])
            os.execve('/usr/bin/python3',['/usr/bin/python3','-B',str(Path(__file__).resolve()),
                '--environment-restored','--construction',str(args.construction),
                '--construction-sha256',args.construction_sha256,'--node',args.node,*ros_args],process['environment'])
        require(dict(os.environ)==process['environment'] and os.getcwd()==process['cwd'],'exact startup environment/cwd differs')
        require(os.environ.get('PYTHONNOUSERSITE')=='1' and os.environ.get('PYTHONDONTWRITEBYTECODE')=='1',
            'original system numerical-library policy changed')
        modules=session.load_modules()
        runtime=rebind(modules['runtime'],W=session.workspace,PRODUCTION=tuple(sorted(session.production)))
        cache=runtime.caches(modules['base'])
        require(not any(row['usable'] for row in cache),'usable changed-target cache exists')
        out=Path(construction['output']);session.storage(out/'probe')
        require(Path(os.environ['ROS_LOG_DIR']).resolve().parent==out,'external per-node ROS log binding differs')
        sys.path[0]=process['original_sys_path0']
        preloaded={}
        if args.node=='/remote_grasp6d_node':
            import alicia_grasp_modes
            for module in ('observe_return_visual','reachable_observation_views'):
                relative=P+'src/alicia_grasp_modes/'+module+'.py'
                modules['preloader'].load_verified_module('alicia_grasp_modes.'+module,
                    session.workspace/relative,session.sources[relative])
                preloaded[relative]=session.sources[relative]
        fields=(Path('/proc')/str(os.getpid())/'stat').read_text().rsplit(')',1)[1].split()
        command=(Path('/proc')/str(os.getpid())/'cmdline').read_bytes()
        receipt=dict(kind='compute_owner_source_start',release_label=session.data['release_label'],
            node=args.node,recorded_ns=time.time_ns(),construction=str(args.construction),
            construction_sha256=args.construction_sha256,
            node_identity=dict(pid=os.getpid(),parent_pid=int(fields[1]),start_ticks=fields[19],
                cmdline_sha256=hashlib.sha256(command).hexdigest()),source_sha256=session.sources,
            cache=cache,run_path=process['run_path'],entry=process['entry'],source_cache_process_proof=True,
            exact_source_preloads=preloaded,memory_attestation=False,
            original_main_entered=False,ros_readiness_proven=False)
        session.save(out/(args.node.lstrip('/')+'_source_start.json'),receipt)
    # Historical bytes are released before the long-running original main.
    sys.argv=[process['run_path'],*ros_args]
    runpy.run_path(process['run_path'],run_name='__main__')


if __name__=='__main__':
    main()
