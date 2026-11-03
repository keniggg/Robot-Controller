"""Start only the reviewed GEN59 remote after exact old-target stop and source install.

Does not stop anything, change mode, publish motion or accept the deployment.
The separate acceptance helper rechecks the fresh instance and remote service.
"""
import ast
import json
import os
from pathlib import Path
import re
import subprocess
import time
import gen59_remote_reload_common_20261022 as c


def main():
    p=c.parser(__doc__)
    for key in ('release','stop-receipt'):
        p.add_argument('--'+key,type=Path,required=True); p.add_argument('--'+key+'-sha256',required=True)
    for key in ('starter-sha256','wrapper-sha256','acceptance-sha256'):
        p.add_argument('--'+key,required=True)
    args=p.parse_args(); ctx=c.context(args,__file__); out=args.output.resolve()
    c.storage(out); c.require(out==c.O/'runtime_reload_r1' and not out.exists(),'single-use GEN59 reload output required')
    stop=args.stop_receipt.resolve(strict=True)
    c.require(c.O in stop.parents and c.sha(stop)==args.stop_receipt_sha256,'stop receipt changed')
    receipt=c.read(stop); c.verify_stopped(ctx,receipt)
    ctx['idle_reference']=dict(names=receipt['names'],positions=receipt['positions'])
    release=c.source_release(ctx,args.release,args.release_sha256)
    c.require(receipt['candidate_source_sha256']==release['source_sha256']
              and receipt['full_regression_sha256']==release['actual_regression_sha256'], 'stopped and installed candidates differ')
    helpers={str(c.W/'.runtime/start_gen59_remote_20261022.sh'):args.starter_sha256,
             str(c.W/'.runtime/start_gen59_remote_20261022.py'):args.wrapper_sha256,
             str(c.W/'.runtime/verify_gen59_remote_reload_20261022.py'):args.acceptance_sha256}
    ctx['pins'].update(helpers); ctx['pins'][str(stop)]=args.stop_receipt_sha256
    c.recheck(ctx,release['source_sha256'])
    out.mkdir(); (out/'remote_ros').mkdir(); os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    result=dict(passed=False,remote_started=False,acceptance_passed=False,deployment_passed=False,
        generation='GEN59',stop_receipt=str(stop),stop_receipt_sha256=args.stop_receipt_sha256,
        source_manifest=str(c.RELEASE),source_manifest_sha256=args.release_sha256,
        previous_deployment=str(c.PREVIOUS),previous_deployment_sha256=c.PREVIOUS_SHA,
        helper_path=str(Path(__file__).resolve()),helper_sha256=c.sha(__file__),common_sha256=c.sha(c.__file__),
        input_sha256=ctx['pins'],motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,
        torque_disable_commands_sent=False,physical_grasp_verified=False,new_launch_attempted=False)
    try:
        import rosgraph
        import rospy
        from std_srvs.srv import Trigger
        from alicia_grasp_modes.selection import parse_selection
        rospy.init_node('gen59_remote_reload',anonymous=True,disable_rosout=True)
        master=rosgraph.Master(rospy.get_name())
        before=c.graph(ctx,rospy,master,auxiliaries=receipt['auxiliary_nodes'])
        idle=c.sample_idle(ctx,rospy,master,receipt['selection'],receipt['epoch_ns'])
        c.same_parameters(receipt['parameter_values_after_cleanup'],c.fingerprints(ctx,master))
        def absent():
            c.verify_stopped(ctx,receipt)
            c.graph(ctx,rospy,master,auxiliaries=receipt['auxiliary_nodes'])
            try:
                master.lookupNode(c.REMOTE)
            except rosgraph.MasterError:
                return
            raise RuntimeError('remote name reappeared; no duplicate launch')
        absent()
        params={key:master.getParam(key) for key in ctx['checker'].PARAMETERS}
        c.require(params==ctx['config']['remote_parameters'],'current remote parameters differ from original expanded launch')
        grasp=master.getParam('/grasp')
        c.require(isinstance(grasp,dict) and bool(grasp),'authoritative grasp mapping missing')
        grasp_digest=c.sha_value(grasp)
        c.require(grasp_digest==receipt['parameter_values_before']['/grasp']['value_sha256'],'grasp config changed')
        unit='alicia-gen59-remote-'+str(time.time_ns())
        construction=dict(schema=1,generation='GEN59',kind='GEN59_preserved_parameters_remote_construction',
            output=str(out),unit=unit,entry=c.ENTRY,release=str(c.RELEASE),release_sha256=args.release_sha256,
            source_sha256=release['source_sha256'],helper_sha256=helpers,
            parameters=params,master_grasp=grasp,master_grasp_sha256=grasp_digest,
            original_startup_configuration=str(c.S/'startup_configuration.json'),
            original_startup_configuration_sha256=c.sha(c.S/'startup_configuration.json'),
            stop_receipt=str(stop),stop_receipt_sha256=args.stop_receipt_sha256,
            selection=idle['selection'],epoch_ns=idle['epoch_ns'],recorded_ns=time.time_ns(),
            no_parameter_value_writes=True,parameters_must_match_before_and_after_start=True)
        construction_path=out/'construction.json'; c.save(construction_path,construction)
        command=['systemd-run','--user','--collect','--unit='+unit,'--property=Type=exec','--property=Restart=no',
            '--property=WorkingDirectory='+str(c.W),'--property=StandardOutput=append:'+str(out/'remote.log'),
            '--property=StandardError=append:'+str(out/'remote.log'),'--setenv=ROS_LOG_DIR='+str(out/'remote_ros'),
            '/bin/bash',str(c.W/'.runtime/start_gen59_remote_20261022.sh'),
            '--construction',str(construction_path),'--construction-sha256',c.sha(construction_path),'--entry',c.ENTRY]
        result.update(unit=unit,command=command,construction=str(construction_path),construction_sha256=c.sha(construction_path),
            idle_before=idle,preserved_before=ctx['preserved'],auxiliary_nodes=receipt['auxiliary_nodes'],new_launch_attempted=True)
        c.save(out/'launch_intent.json',result)
        c.recheck(ctx,release['source_sha256']); absent()
        subprocess.run(command,check=True,timeout=15.)
        rospy.wait_for_service('/grasp_mode/reset_remote',timeout=40.)
        remote=c.identity(ctx,master,rospy.get_name(),c.REMOTE)
        unit_props=ctx['v'].properties(unit)
        c.require(remote['pid']==int(unit_props['MainPID']) and remote!=ctx['nodes'][c.REMOTE], 'new remote is not direct unit MainPID')
        c.graph(ctx,rospy,master,remote,unit,receipt['auxiliary_nodes'])
        c.sample_idle(ctx,rospy,master,idle['selection'],idle['epoch_ns'])
        response=rospy.ServiceProxy('/grasp_mode/reset_remote',Trigger)()
        c.require(response.success,'remote reset rejected: '+response.message)
        ack=json.loads(response.message)
        c.require(ack.get('ready') is True and parse_selection(ack)==idle['selection'],'remote reset acknowledged wrong selection')
        c.require(master.getParam('/grasp')==grasp and
                  {key:master.getParam(key) for key in ctx['checker'].PARAMETERS}==params,'construction parameters changed')
        deadline=time.monotonic()+3.; lines=[]; prefix=b''
        while time.monotonic()<deadline:
            prefix=(out/'remote.log').read_bytes()
            lines=[line for line in prefix.decode(errors='replace').splitlines() if '[RemoteGrasp6D] authoritative /grasp loaded ' in line]
            if lines:
                break
            time.sleep(.05)
        c.require(len(lines)==1,'expected unique authoritative grasp startup marker')
        match=re.search(r'authoritative /grasp loaded key_count=(\d+) contact_height_registration_margin_enabled=(\S+) handeye_translation_max_error_m=(\S+)',lines[0])
        c.require(match is not None,'startup mapping marker format differs')
        loaded=dict(key_count=int(match.group(1)),contact_height_registration_margin_enabled=ast.literal_eval(match.group(2)),
                    handeye_translation_max_error_m=ast.literal_eval(match.group(3)))
        c.require(loaded==dict(key_count=len(grasp),contact_height_registration_margin_enabled=grasp.get('contact_height_registration_margin_enabled'),
                              handeye_translation_max_error_m=grasp.get('handeye_translation_max_error_m')),'startup grasp mapping differs')
        frozen=out/'remote_startup_prefix.log'
        with frozen.open('xb') as stream:
            stream.write(prefix); stream.flush(); os.fsync(stream.fileno())
        c.require(frozen.read_bytes()==prefix and (out/'remote.log').read_bytes()[:len(prefix)]==prefix,'startup prefix changed')
        attestation_path=out/'runtime_import_attestation.json'; attestation=c.read(attestation_path)
        c.verify_attestation(ctx,attestation,remote,construction_path,release)
        binding=dict(schema=1,node_name=c.REMOTE,master_uri='http://localhost:11311',parameters=params,
            allowed_backends=ctx['config']['allowed_backends'],client_contract_source=ctx['config']['client_contract_source'],
            node_identity=attestation['node_identity'],construction_evidence_review_required=True,
            ready_health_not_implied_by_startup_binding=True,
            evidence=[dict(path=str(p),sha256=c.sha(p)) for p in
                      (construction_path,c.S/'startup_configuration.json',c.RELEASE,attestation_path,frozen)],
            construction_provenance=dict(kind='fresh_remote_only_preserved_expanded_launch_parameters',
                unit=unit,live_parameters_do_not_define_expected_values=True,source_preloaded_before_original_main=True),created_ns=time.time_ns())
        binding_path=out/'remote_startup_binding.json'; c.save(binding_path,binding)
        result.update(remote_started=True,passed=True,remote_identity=remote,remote_service=unit_props,reset_ack=ack,
            runtime_import_attestation=str(attestation_path),runtime_import_attestation_sha256=c.sha(attestation_path),
            startup_configuration=dict(loaded=loaded,master_grasp_sha256=grasp_digest,frozen_log_prefix=str(frozen),
                                       frozen_log_prefix_sha256=c.sha(frozen)),
            remote_startup_binding=str(binding_path),remote_startup_binding_sha256=c.sha(binding_path),
            idle_after=c.sample_idle(ctx,rospy,master,idle['selection'],idle['epoch_ns']),recorded_ns=time.time_ns())
        c.graph(ctx,rospy,master,remote,unit,receipt['auxiliary_nodes']); c.recheck(ctx,release['source_sha256'])
        c.save(out/'result.json',result)
    except Exception as exc:
        result.update(passed=False,error=repr(exc),recorded_ns=time.time_ns())
        c.save(out/'failure.json',result)
        raise


if __name__=='__main__':
    main()
