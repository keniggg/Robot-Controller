"""Single-use GEN62 startup adapter; execute only a frozen original compute entry.

The receipt proves source/cache/process inputs before original main, not memory
attestation or ROS readiness. Environment restoration execs the same PID once.
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
    if not ok:raise RuntimeError(reason)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--construction',type=Path,required=True)
    parser.add_argument('--construction-sha256',required=True)
    parser.add_argument('--node',required=True)
    parser.add_argument('--environment-restored',action='store_true')
    args,ros_args=parser.parse_known_args()
    require(sha(args.construction)==args.construction_sha256,'construction changed')
    construction=json.loads(args.construction.read_bytes());process=construction['processes'][args.node]
    require(ros_args==process['ros_args'],'original ROS argument/remapping changed')
    bundle_path=Path(construction['bundle'])
    require(sha(bundle_path)==construction['bundle_sha256'],'bundle changed')
    bundle=json.loads(bundle_path.read_bytes());require(bundle['deployment_ready'] is True,'final bundle required')
    require(all(sha(p)==h for p,h in bundle['pins'].items()),'pinned source/helper evidence changed')
    if not args.environment_restored:
        os.chdir(process['cwd'])
        os.execve('/usr/bin/python3',['/usr/bin/python3','-B',str(Path(__file__).resolve()),
            '--environment-restored','--construction',str(args.construction),'--construction-sha256',args.construction_sha256,
            '--node',args.node,*ros_args],process['environment'])
    require(dict(os.environ)==process['environment'] and os.getcwd()==process['cwd'],'exact startup environment/cwd differs')
    require(os.environ.get('PYTHONNOUSERSITE')=='1' and os.environ.get('PYTHONDONTWRITEBYTECODE')=='1',
        'original system numerical-library policy changed')
    root=Path(__file__).resolve().parents[1]
    require(all(sha(root/p)==h for p,h in bundle['source_sha256'].items()),'installed source closure differs')
    require(all(sha(root/p)==h for p,h in bundle['load_resolution_sha256'].items()),'catkin load resolution differs')
    # Cache classifier is the unchanged GEN60 primitive. Importing this
    # adapter has no ROS or production-node effects.
    import gen62_compute_runtime_20261022 as adapter
    common=adapter.load(adapter.COMMON,adapter.COMMON_SHA,'gen62_start_cache_classifier')
    cache=adapter.caches(common);require(not any(row['usable'] for row in cache),'usable changed-target cache exists')
    out=Path(construction['output']);adapter.storage(out/'probe')
    require(Path(os.environ['ROS_LOG_DIR']).resolve().parent==out,'external per-node ROS log binding differs')
    sys.path[0]=process['original_sys_path0']
    preloaded={}
    if args.node=='/remote_grasp6d_node':
        # Preserve GEN59's existing direct-source guarantee for these two
        # geometry modules; their ordinary caches need not be trusted.
        import alicia_grasp_modes
        previous=adapter.load(adapter.START59,adapter.START59_SHA,'gen62_retained_gen59_source_loader')
        for module in ('observe_return_visual','reachable_observation_views'):
            relative=adapter.P+'src/alicia_grasp_modes/'+module+'.py'
            previous.load_verified_module('alicia_grasp_modes.'+module,root/relative,bundle['source_sha256'][relative])
            preloaded[relative]=bundle['source_sha256'][relative]
    fields=(Path('/proc')/str(os.getpid())/'stat').read_text().rsplit(')',1)[1].split()
    command=(Path('/proc')/str(os.getpid())/'cmdline').read_bytes()
    receipt=dict(kind='GEN62_source_cache_process_start',node=args.node,recorded_ns=time.time_ns(),
        construction=str(args.construction),construction_sha256=args.construction_sha256,
        node_identity=dict(pid=os.getpid(),parent_pid=int(fields[1]),start_ticks=fields[19],
            cmdline_sha256=hashlib.sha256(command).hexdigest()),source_sha256=bundle['source_sha256'],
        cache=cache,run_path=process['run_path'],entry=process['entry'],source_cache_process_proof=True,
        exact_source_preloads=preloaded,
        memory_attestation=False,original_main_entered=False,ros_readiness_proven=False)
    adapter.save(out/(args.node.lstrip('/')+'_source_start.json'),receipt)
    # Original catkin relay for task/unknown; original source __main__ for
    # remote replaces only its obsolete GEN59-specific bootstrap wrapper.
    sys.argv=[process['run_path'],*ros_args]
    runpy.run_path(process['run_path'],run_name='__main__')


if __name__=='__main__':
    main()
