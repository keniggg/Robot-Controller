"""Read-only GEN60 gateway-only acceptance; original GEN59 remote retained.

This verifies process/source-load evidence, not Python memory introspection.
No mode selection, motion or reset service is invoked.
"""
import os
from pathlib import Path
import time
import gen60_gateway_reload_common_20261022 as c


def main():
    p=c.parser(__doc__)
    for key in ('release','reload'):
        p.add_argument('--'+key,type=Path,required=True);p.add_argument('--'+key+'-sha256',required=True)
    args=p.parse_args();ctx=c.context(args,__file__);c.reviewed_helpers(ctx,args)
    release=c.source_release(ctx,args.release,args.release_sha256)
    start=c.bind(ctx,args.reload,args.reload_sha256)
    c.require(args.reload.resolve()==c.O/'runtime_reload_r1/result.json' and start['passed']
        and start['kind']=='GEN60_gateway_source_install_and_respawn' and start['source_installed']
        and start['gateway_reloaded'] and start['old_gateway_exited'] and not start['acceptance_passed']
        and start['source_manifest_sha256']==args.release_sha256 and start['helper_sha256']==c.helper_map(),
        'exact successful gateway reload receipt required')
    for p,h in start['input_sha256'].items():
        c.require(c.sha(p)==h,'reload input changed');ctx['pins'][p]=h
    pre=c.bind(ctx,start['preflight'],start['preflight_sha256'])
    c.require(pre['passed'] and start['before']==pre['node_identities']==ctx['nodes']
        and start['preserved_before']==start['preserved_after']==ctx['preserved']
        and pre['parameter_snapshot']==start['parameter_snapshot'],'reload predecessor/preflight differs')
    requests=start['shutdown_requests'];old=ctx['nodes'][c.GATEWAY];gateway=start['new_gateway_identity']
    c.require(len(requests)==1 and requests[0]['name']==c.GATEWAY and requests[0]['identity']==old
        and requests[0]['attempted'] is True and gateway!=old and gateway['parent_pid']==old['parent_pid'],
        'single exact gateway replacement not proven')
    c.require(ctx['legacy'].exited(old),'original gateway still present')
    out=args.output.resolve();c.storage(out)
    c.require(out==c.DEPLOYMENT.parent and not out.exists(),'new fixed GEN60 acceptance output required')
    out.mkdir();os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    ctx['expected_gateway']=gateway;ctx['idle_reference']=dict(names=pre['names'],positions=pre['positions'])
    dep=ctx['previous']
    result=dict(generation='GEN60',deployment_passed=False,whole_patch_deployed=False,
        deployment_kind='original_parent_gateway_respawn_sixteen_preserved',full_restart=False,all_processes_replaced=False,
        source_manifest=str(c.RELEASE),source_manifest_sha256=args.release_sha256,
        source_sha256=release['source_sha256'],test_summary=release['test_summary'],
        previous_deployment=str(c.PREVIOUS),previous_deployment_sha256=c.PREVIOUS_SHA,
        post_attempt_idle=str(c.POST),post_attempt_idle_sha256=c.POST_SHA,
        reload_evidence=str(args.reload.resolve()),reload_sha256=args.reload_sha256,
        input_sha256=ctx['pins'],helper_sha256=c.helper_map(),
        motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,
        torque_disable_commands_sent=False,physical_grasp_verified=False,
        fresh_scene_and_dependency_gate_required_before_automatic_attempt=True,automatic_attempt_authorized=False)
    try:
        import rospy
        import rosgraph
        from sensor_msgs.msg import Image
        rospy.init_node('gen60_gateway_acceptance',anonymous=True,disable_rosout=True)
        master=rosgraph.Master(rospy.get_name())
        before=c.graph(ctx,rospy,master,gateway=gateway,auxiliaries=start['ownership']['auxiliary_nodes'])
        c.require(before['node_identities']==start['full_runtime_nodes'],'accepted reload inventory changed')
        c.same_parameters(pre['parameter_snapshot'],c.parameters(master,out,'initial'),out,'initial')
        proof=c.source_load_proof(ctx,gateway,release['source_sha256'],pre['gateway_original_parent'])
        c.same_gateway_load_contract(pre['gateway_source_load_proof'],proof)
        c.require(proof==start['gateway_source_load_proof'],'source load proof changed')
        services=c.service_owners(ctx,master)
        idle=c.sample_idle(ctx,rospy,master,pre['selection'],pre['epoch_ns'])
        camera={}
        for key,topic in (('rgb','/supervisor/camera/color/image_raw'),('depth','/supervisor/camera/depth/image_raw')):
            msg=rospy.wait_for_message(topic,Image,timeout=4.)
            camera[key]=ctx['v'].camera_sample(msg,rospy.Time.now().to_nsec())
        health=c.fresh_remote_health(ctx,out/'remote_readiness')
        final_idle=c.sample_idle(ctx,rospy,master,idle['selection'],idle['epoch_ns'])
        after=c.graph(ctx,rospy,master,gateway=gateway,auxiliaries=before['auxiliary_nodes'])
        c.require(before==after and ctx['legacy'].exited(old),'ownership changed during acceptance')
        c.same_parameters(pre['parameter_snapshot'],c.parameters(master,out,'final'),out,'final');c.recheck(ctx,release['source_sha256'])
        nodes=after['node_identities']
        result.update(final_idle,deployment_passed=True,whole_patch_deployed=True,
            gateway_reloaded=True,gateway_original_parent_preserved=True,other_sixteen_processes_preserved=True,
            hardware_processes_preserved=True,epoch_preserved=True,replaced_count=1,preserved_count=16,
            unit=dep['unit'],core_service=after['core_service'],master_process=after['master_process'],
            simulation_unit=dep['simulation_unit'],simulation_service=after['simulation_service'],
            remote_unit=dep['remote_unit'],remote_service=dep['remote_service'],
            before=ctx['nodes'],after={n:nodes[n] for n in ctx['v'].NODES},
            replaced_before={c.GATEWAY:old},replaced_after={c.GATEWAY:gateway},new_gateway_identity=gateway,
            preserved_before=ctx['preserved'],preserved_after=ctx['preserved'],
            full_runtime_nodes=nodes,all_registered_nodes=nodes,final_all_registered_nodes=nodes,
            core_eleven_after_is_compatibility_view=True,auxiliary_nodes=after['auxiliary_nodes'],ownership=after,
            gui_owning_manual_ack=dict(callerid='/alicia_supervisor_gui',manual=True),
            gateway_source_load_proof=proof,gateway_service_owners=services,parameter_snapshot=pre['parameter_snapshot'],
            control_revision=67,control_unchanged=True,
            startup_configuration=dep['startup_configuration'],startup_configuration_sha256=dep['startup_configuration_sha256'],
            runtime_import_attestation=dep['runtime_import_attestation'],runtime_import_attestation_sha256=dep['runtime_import_attestation_sha256'],
            remote_dependency=health,composite_runtime=dict(gateway_generation='GEN60',remote_generation='GEN59',
                changed_production_sources=[c.contract.SELECTOR],changed_source_consumers=[c.GATEWAY],
                remote_preload_attestation_retained=True,remote_reloaded=False),
            camera=camera,camera_frame_check_does_not_grant_scene_clearance=True,recorded_ns=time.time_ns())
        c.validate_partition(ctx,result);c.save(out/'result.json',result)
    except Exception as exc:
        result.update(deployment_passed=False,whole_patch_deployed=False,error=repr(exc),recorded_ns=time.time_ns())
        c.save(out/'failure.json',result);raise


if __name__=='__main__':main()
