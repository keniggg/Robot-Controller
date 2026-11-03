"""Load exact reviewed source in the new remote PID, attest it, then run its original entry.

This is a startup adapter, not an import hook or new planning path. Only the two
pure geometry modules and original entry are preloaded. No stale pyc is used for
these three changed files. Their original class and spin loop remain intact.
"""
import argparse
import hashlib
import importlib.machinery
import importlib.util
import json
import os
from pathlib import Path
import sys
import time


def require(ok, why):
    if not ok:
        raise RuntimeError(why)


def load_verified_module(name, path, expected):
    """Preserve importlib/sys.modules identity while bypassing stale bytecode."""
    path=Path(path).resolve(strict=True); body=path.read_bytes()
    require(hashlib.sha256(body).hexdigest()==expected,'module bytes changed')
    require(name not in sys.modules,'changed module loaded before exact source attestation: '+name)
    class VerifiedSourceLoader(importlib.machinery.SourceFileLoader):
        def get_code(self, fullname):
            current=Path(self.path).read_bytes()
            require(current==body,'source changed during module load')
            return compile(current,self.path,'exec',dont_inherit=True)
    loader=VerifiedSourceLoader(name,str(path))
    spec=importlib.util.spec_from_file_location(name,path,loader=loader)
    module=importlib.util.module_from_spec(spec); sys.modules[name]=module
    try:
        loader.exec_module(module)
    except BaseException:
        sys.modules.pop(name,None)
        raise
    if '.' in name:
        parent,attribute=name.rsplit('.',1)
        setattr(sys.modules[parent],attribute,module)
    require(Path(module.__file__).resolve()==path and hashlib.sha256(path.read_bytes()).hexdigest()==expected,
            'loaded module path/source changed')
    return module


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--construction',type=Path,required=True)
    p.add_argument('--construction-sha256',required=True)
    p.add_argument('--entry',required=True)
    args,ros_args=p.parse_known_args()
    sha=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
    w=Path(__file__).resolve().parents[1]
    raw=args.construction.read_bytes()
    require(hashlib.sha256(raw).hexdigest()==args.construction_sha256,'construction changed')
    cfg=json.loads(raw); out=Path(cfg['output'])
    require(cfg['entry']==args.entry=='src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py','entry differs')
    require(sha(__file__)==cfg['helper_sha256'][str(Path(__file__).resolve())],'startup wrapper changed')
    require(all(sha(p)==h for p,h in cfg['helper_sha256'].items()),'startup helper changed')
    require(sha(cfg['release'])==cfg['release_sha256'],'release changed')
    release=json.loads(Path(cfg['release']).read_bytes())
    require(release['source_sha256']==cfg['source_sha256'],'startup source mapping differs')
    require(all(sha(w/p)==h for p,h in cfg['source_sha256'].items()),'startup source changed')
    logs=Path(os.environ['ROS_LOG_DIR']).resolve(strict=True)
    require(logs==out/'remote_ros' and logs.stat().st_dev==Path('/media/zhuyupei/SanStand').stat().st_dev,
            'external startup logs differ')
    require(os.environ.get('PYTHONDONTWRITEBYTECODE')=='1' and os.environ.get('PYTHONNOUSERSITE')=='1',
            'startup Python environment differs')
    # Package import resolves only its original __init__; source for the three
    # changed modules is executed directly from the bytes whose hash is sealed.
    import alicia_grasp_modes
    loaded={}
    declarations=[('alicia_grasp_modes.observe_return_visual','src/alicia_grasp_modes/src/alicia_grasp_modes/observe_return_visual.py'),
        ('alicia_grasp_modes.reachable_observation_views','src/alicia_grasp_modes/src/alicia_grasp_modes/reachable_observation_views.py'),
        ('_gen59_reviewed_remote_entry',args.entry)]
    for name,relative in declarations:
        path=w/relative
        module=load_verified_module(name,path,cfg['source_sha256'][relative])
        loaded[relative]=dict(module=name,path=str(path),sha256=sha(path),
                             compiled_from_verified_source_bytes=True)
    fields=(Path('/proc')/str(os.getpid())/'stat').read_text().rsplit(')',1)[1].split()
    command=(Path('/proc')/str(os.getpid())/'cmdline').read_bytes()
    attestation=dict(schema=1,kind='GEN59_same_process_source_preload',recorded_ns=time.time_ns(),
        node_identity=dict(pid=os.getpid(),parent_pid=int(fields[1]),start_ticks=fields[19],cmdline_sha256=hashlib.sha256(command).hexdigest()),
        construction=str(args.construction),construction_sha256=args.construction_sha256,
        release=cfg['release'],release_sha256=cfg['release_sha256'],loaded_sources=loaded,
        planning_executed=False,physical_grasp_verified=False)
    encoded=(json.dumps(attestation,ensure_ascii=False,indent=2,allow_nan=False)+'\n').encode()
    path=out/'runtime_import_attestation.json'
    with path.open('xb') as stream:
        stream.write(encoded); stream.flush(); os.fsync(stream.fileno())
    require(path.read_bytes()==encoded,'attestation readback differs')
    # Equivalent to the production file's __main__ block, after exact preload.
    sys.argv=[str(w/args.entry),*ros_args]
    entry=sys.modules['_gen59_reviewed_remote_entry']
    entry.rospy.init_node('remote_grasp6d_node')
    entry.ModeAwareRemoteGrasp6DNode().spin()


if __name__=='__main__':
    main()
