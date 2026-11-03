"""Bind a new idle GEN56 remote instance to its reviewed startup configuration.

The earlier cold-start configuration is immutable evidence. Current parameters
must match before and after reload; they never define the expected values. This
read-only helper does not grant dependency readiness or start any prediction.
"""
import argparse
import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import time

from gen56_idle_reload_common_20261020 import (
    W,N,BASE_SHA,PREVIOUS_SHA,PARAMETERS,sha,read,require,storage,write,process)

OLD = N/'verification/gen55_remote_readiness_diagnosis_r1/startup_binding.json'
OLD_SHA = '19260cf0dc61f63d22029ceed5202df5a9f9918948413f790ccb1d1378308243'
CHECKER = W/'.runtime/verify_active_remote_readiness_20261020.py'
CHECKER_SHA = 'eaa51eaf5b636b74d9957798bbf3b0407a0d25d5c6b11097c02b22a5e97948e5'
BASELINE = W/'.runtime/gen56_baseline_source'
INITIALIZERS = {
    'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py': 'RemoteGrasp6DNode',
    'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py': 'ModeAwareRemoteGrasp6DNode'}


def constructor(raw, class_name):
    tree = ast.parse(raw)
    cls = next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==class_name)
    init = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='__init__')
    return ast.dump(init,include_attributes=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('deployment','preflight','release'):
        parser.add_argument('--'+name,type=Path,required=True)
        parser.add_argument('--'+name+'-sha256',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    output=args.output.resolve();storage(output)
    require(not output.exists(),'output already exists')
    refs={name:getattr(args,name).resolve(strict=True) for name in ('deployment','preflight','release')}
    require(all(sha(p)==getattr(args,k+'_sha256') for k,p in refs.items()),'reload input changed')
    require(sha(OLD)==OLD_SHA and sha(CHECKER)==CHECKER_SHA,'reviewed startup/checker changed')
    old=read(OLD);deployment=read(refs['deployment']);preflight=read(refs['preflight']);release=read(refs['release'])
    require(deployment['deployment_passed'] is True and deployment['whole_patch_deployed'] is True and
        deployment['manual_mode'] is True and deployment['task_active'] is False and
        deployment['actions_active'] is False and preflight['passed'] is True,
        'accepted idle reload required')
    require(deployment['source_manifest_sha256']==sha(refs['release']) and
        deployment['source_sha256']==release['source_sha256'] and
        deployment['deployment_preflight_sha256']==sha(refs['preflight']) and
        deployment['previous_deployment_sha256']==PREVIOUS_SHA and
        release['generation']=='GEN56' and release['baseline_release_sha256']==BASE_SHA,
        'reload source/baseline mismatch')
    require({k:preflight['runtime_parameters'][k] for k in old['parameters']}==old['parameters'],
        'construction parameters changed before reload')
    require(deployment['selection']==preflight['selection'],'selection changed during reload')
    # Only historical source references are relocated to an identical frozen
    # copy. Their original path and digest remain recorded in the old binding.
    evidence=[];relocated=[]
    spec=importlib.util.spec_from_file_location('gen56_remote_readiness_checker',CHECKER)
    checker=importlib.util.module_from_spec(spec);spec.loader.exec_module(checker)
    for ref in old['evidence']:
        row=copy.deepcopy(ref);path=Path(row['path'])
        if W/'src' in path.parents:
            row['path']=str(BASELINE/path.relative_to(W))
            relocated.append(dict(original=ref,frozen=row))
        checker.reference_bytes(row);evidence.append(row)
    initializer_proofs={}
    for relative,class_name in INITIALIZERS.items():
        baseline=BASELINE/relative;current=W/relative
        require(sha(current)==release['source_sha256'][relative], 'loaded source changed')
        require(constructor(baseline.read_bytes(),class_name)==constructor(current.read_bytes(),class_name),
            'remote constructor changed; separate configuration review required')
        initializer_proofs[relative]=dict(baseline_sha256=sha(baseline),current_sha256=sha(current),
            initializer_ast_equal=True)
        evidence.append(dict(path=str(current),sha256=sha(current)))
    for p in (*refs.values(),OLD,Path(__file__).resolve(),
              W/'.runtime/gen56_observation_measurement_consumers_20261020.launch',
              W/'.runtime/start_gen56_observation_measurement_consumers_20261020.sh'):
        evidence.append(dict(path=str(p),sha256=sha(p)))
    # The new launch is exactly the reviewed previous launch except generation
    # file labels; neither launch supplies a remote client configuration override.
    import xml.etree.ElementTree as ET
    new_launch=W/'.runtime/gen56_observation_measurement_consumers_20261020.launch'
    prior_launch=W/'.runtime/gen55_observation_measurement_consumers_20261019.launch'
    require(ET.tostring(ET.parse(new_launch).getroot())==ET.tostring(ET.parse(prior_launch).getroot()),
        'launch parameter inheritance changed')
    identity=deployment['after']['/remote_grasp6d_node']
    require(process(identity['pid'])==identity,'new remote instance changed')
    cmd=(Path('/proc')/str(identity['pid'])/'cmdline').read_bytes()
    identity=dict(identity,cmdline_sha256=hashlib.sha256(cmd).hexdigest())
    binding=dict(schema=1,node_name=old['node_name'],master_uri=old['master_uri'],
        parameters=copy.deepcopy(old['parameters']),allowed_backends=old['allowed_backends'],
        client_contract_source=old['client_contract_source'],node_identity=identity,evidence=evidence,
        construction_evidence_review_required=True,ready_health_not_implied_by_startup_binding=True,
        construction_provenance=dict(prior_binding=str(OLD),prior_binding_sha256=OLD_SHA,
            frozen_configuration='exact original cold-start configuration, checked before and after new idle reload',
            initializer_proofs=initializer_proofs,relocated_historical_sources=relocated,
            scope_limit='reviewed construction chain; current parameters alone do not prove constructor values'),
        created_ns=time.time_ns())
    observed=checker.observe(binding);checker.check_binding(binding,observed)
    require(observed['selection']==deployment['selection'],'selection changed before binding')
    require(all(sha(p)==getattr(args,k+'_sha256') for k,p in refs.items()),'reload input changed')
    output.mkdir();write(output/'startup_binding.json',binding)
    checker.load_binding(output/'startup_binding.json',sha(output/'startup_binding.json'))
    write(output/'result.json',dict(passed=True,dependency_health_checked=False,
        binding=str(output/'startup_binding.json'),binding_sha256=sha(output/'startup_binding.json'),
        checker_sha256=CHECKER_SHA,observation=checker.public_observation(observed),
        motion_commands_sent=False,recorded_ns=time.time_ns()))
    print(json.dumps(dict(passed=True,binding=str(output/'startup_binding.json'),
        sha256=sha(output/'startup_binding.json')),ensure_ascii=False))


if __name__=='__main__':
    main()
