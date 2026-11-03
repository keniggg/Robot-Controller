"""Read-only independent GEN59 remote-only acceptance. Never starts an attempt.

Preserves all 16 original nodes, GUI, master, core/simulation units and reference
epoch. Checks fresh SDK feedback and the current new process's remote dependency.
"""
import os
from pathlib import Path
import time
import gen59_remote_reload_common_20261022 as c


def main():
    p=c.parser(__doc__)
    for key in ('release','reload'):
        p.add_argument('--'+key,type=Path,required=True); p.add_argument('--'+key+'-sha256',required=True)
    args=p.parse_args(); ctx=c.context(args,__file__); out=args.output.resolve()
    c.storage(out); c.require(out==c.O/'runtime_reload_r1/acceptance' and not out.exists(),'new fixed acceptance output required')
    reload_path=args.reload.resolve(strict=True)
    c.require(reload_path==out.parent/'result.json' and c.sha(reload_path)==args.reload_sha256,'reload evidence changed')
    start=c.read(reload_path); release=c.source_release(ctx,args.release,args.release_sha256)
    c.require(start['passed'] and start['remote_started'] and start['acceptance_passed'] is False
              and start['source_manifest_sha256']==args.release_sha256,'new remote startup not proven')
    ctx['pins'].update(start['input_sha256']); ctx['pins'][str(reload_path)]=args.reload_sha256
    construction_path=Path(start['construction']); construction=c.read(construction_path)
    c.require(c.sha(construction_path)==start['construction_sha256']
              and construction['helper_sha256'][str(Path(__file__).resolve())]==args.helper_sha256,'startup/acceptance helper binding differs')
    stop_path=Path(start['stop_receipt']); stop=c.read(stop_path)
    c.require(c.sha(stop_path)==start['stop_receipt_sha256'] and stop['passed'] and stop['remote_stopped']
              and stop['node_identities']==ctx['nodes'] and stop['preserved_node_identities']==ctx['preserved'], 'stopped predecessor differs')
    c.require(ctx['legacy'].exited(ctx['nodes'][c.REMOTE]),'old remote reappeared')
    ctx['idle_reference']=dict(names=stop['names'],positions=stop['positions'])
    ctx['pins'][str(stop_path)]=start['stop_receipt_sha256']
    ctx['pins'][str(construction_path)]=start['construction_sha256']
    out.mkdir(); os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    result=dict(deployment_passed=False,whole_patch_deployed=False,generation='GEN59',full_restart=False,
        all_processes_replaced=False,deployment_kind='GEN59_remote_only_sixteen_processes_preserved',
        dedicated_compute_reloader_compatible=False,remote_only_reloader_compatible=True,
        source_manifest=str(c.RELEASE),source_manifest_sha256=args.release_sha256,
        source_sha256=release['source_sha256'],test_summary=release['test_summary'],
        previous_deployment=str(c.PREVIOUS),previous_deployment_sha256=c.PREVIOUS_SHA,
        reload_evidence=str(reload_path),reload_sha256=args.reload_sha256,
        stop_receipt=str(stop_path),stop_receipt_sha256=start['stop_receipt_sha256'],
        motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,
        torque_disable_commands_sent=False,physical_grasp_verified=False,
        fresh_scene_and_dependency_gate_required_before_automatic_attempt=True,input_sha256=ctx['pins'])
    try:
        import rosgraph
        import rospy
        from sensor_msgs.msg import Image
        rospy.init_node('gen59_remote_acceptance',anonymous=True,disable_rosout=True)
        master=rosgraph.Master(rospy.get_name()); remote=start['remote_identity']; unit=start['unit']
        c.require(ctx['v'].properties(unit)==start['remote_service'],'new remote service changed')
        before=c.graph(ctx,rospy,master,remote,unit,stop['auxiliary_nodes'])
        idle=c.sample_idle(ctx,rospy,master,stop['selection'],stop['epoch_ns'])
        camera={}
        for key,topic in (('rgb','/supervisor/camera/color/image_raw'),('depth','/supervisor/camera/depth/image_raw')):
            message=rospy.wait_for_message(topic,Image,timeout=4.)
            camera[key]=ctx['v'].camera_sample(message,rospy.Time.now().to_nsec())
        c.same_parameters(stop['parameter_values_before'],c.fingerprints(ctx,master))
        c.require(master.getParam('/grasp')==construction['master_grasp'],'authoritative grasp mapping drifted')
        attestation_path=Path(start['runtime_import_attestation'])
        c.require(c.sha(attestation_path)==start['runtime_import_attestation_sha256'],'import attestation changed')
        c.verify_attestation(ctx,c.read(attestation_path),remote,construction_path,release)
        binding_path=Path(start['remote_startup_binding'])
        binding=ctx['checker'].load_binding(binding_path,start['remote_startup_binding_sha256'])
        c.require(binding['parameters']==construction['parameters']==ctx['config']['remote_parameters'], 'remote config differs')
        health_out=out/'remote_readiness'; health_out.mkdir()
        health=ctx['checker'].evaluate(binding,health_out); c.save(health_out/'result.json',health)
        c.require(health['passed'] and health['dependency_ready'],'new current remote dependency is not ready')
        final_idle=c.sample_idle(ctx,rospy,master,idle['selection'],idle['epoch_ns'])
        after=c.graph(ctx,rospy,master,remote,unit,stop['auxiliary_nodes'])
        c.require(before==after and ctx['v'].properties(unit)==start['remote_service'],'ownership changed during acceptance')
        c.recheck(ctx,release['source_sha256'])
        nodes=dict(ctx['preserved'],**{c.REMOTE:remote})
        result.update(final_idle,deployment_passed=True,whole_patch_deployed=True,remote_reloaded=True,
            hardware_processes_preserved=True,other_sixteen_processes_preserved=True,epoch_preserved=True,
            unit=ctx['previous']['unit'],core_service=before['core_service'],master_process=before['master_process'],
            simulation_unit=ctx['previous']['simulation_unit'],simulation_service=before['simulation_service'],
            remote_unit=unit,remote_service=start['remote_service'],new_remote_identity=remote,
            before=ctx['nodes'],after={n:nodes[n] for n in ctx['v'].NODES},
            replaced_before={c.REMOTE:ctx['nodes'][c.REMOTE]},replaced_after={c.REMOTE:remote},
            preserved_before=ctx['preserved'],preserved_after=ctx['preserved'],
            full_runtime_nodes=nodes,all_registered_nodes=nodes,final_all_registered_nodes=nodes,
            core_eleven_after_is_compatibility_view=True,auxiliary_nodes=stop['auxiliary_nodes'],
            ownership=after,gui_owning_manual_ack=dict(callerid='/alicia_supervisor_gui',manual=True),
            control_revision=67,control_unchanged=True,reset_ack=start['reset_ack'],
            startup_configuration=str(construction_path),startup_configuration_sha256=c.sha(construction_path),
            runtime_import_attestation=str(attestation_path),runtime_import_attestation_sha256=c.sha(attestation_path),
            camera=camera,camera_frame_check_does_not_grant_scene_clearance=True,
            remote_dependency=dict(startup_binding=str(binding_path),startup_binding_sha256=c.sha(binding_path),
                readiness=str(health_out/'result.json'),readiness_sha256=c.sha(health_out/'result.json'),
                passed=True,health_not_a_prediction_or_geometry_proof=True),recorded_ns=time.time_ns())
        c.save(out/'result.json',result)
    except Exception as exc:
        result.update(deployment_passed=False,whole_patch_deployed=False,error=repr(exc),recorded_ns=time.time_ns())
        c.save(out/'failure.json',result)
        raise


if __name__=='__main__':
    main()
