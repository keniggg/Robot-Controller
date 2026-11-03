"""Read-only GEN60 gateway reload preflight. No shutdown/install or motion."""
import os
import time
import gen60_gateway_reload_common_20261022 as c


def main():
    args=c.parser(__doc__,evidence=True).parse_args()
    ctx=c.context(args,__file__);c.reviewed_helpers(ctx,args)
    checked=c.frozen_evidence(ctx,args)
    out=args.output.resolve();c.storage(out)
    c.require(c.O in out.parents and not out.exists(),'new GEN60 output required');out.mkdir()
    os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    result=dict(kind='GEN60_gateway_idle_install_readiness',passed=False,generation='GEN60',
        deployment_sha256=c.PREVIOUS_SHA,release_sha256=c.BASE_SHA,helper_sha256=c.helper_map(),
        frozen_evidence={key:dict(path=str(getattr(args,key).resolve()),sha256=getattr(args,key+'_sha256'))
                         for key in ('manifest','full','review')},
        helper_review=str(args.helper_review.resolve()),helper_review_sha256=args.helper_review_sha256,
        input_sha256=ctx['pins'],motion_commands_sent=False,mode_changes_sent=False,
        temperature_queried=False,torque_disable_commands_sent=False,source_installed=False,shutdown_sent=False)
    try:
        import rospy
        import rosgraph
        rospy.init_node('gen60_gateway_preflight',anonymous=True,disable_rosout=True)
        master=rosgraph.Master(rospy.get_name());ctx['expected_gateway']=ctx['nodes'][c.GATEWAY]
        before=c.graph(ctx,rospy,master);parameters=c.parameters(master,out,'before')
        result['parameter_snapshot']=parameters
        proof=c.source_load_proof(ctx,ctx['expected_gateway'],checked['baseline_source_sha256'],allow_usable_cache=True)
        idle=c.sample_idle(ctx,rospy,master)
        services=c.service_owners(ctx,master)
        health=c.fresh_remote_health(ctx,out/'remote_readiness')
        final_idle=c.sample_idle(ctx,rospy,master,idle['selection'],idle['epoch_ns'])
        after=c.graph(ctx,rospy,master,auxiliaries=before['auxiliary_nodes'])
        c.same_parameters(parameters,c.parameters(master,out,'final'),out,'final');c.require(before==after,'preflight ownership changed')
        c.recheck(ctx,checked['baseline_source_sha256'])
        result.update(final_idle,passed=True,node_identities=ctx['nodes'],preserved_node_identities=ctx['preserved'],
            gateway_original_parent=proof['parent_identity'],gateway_source_load_proof=proof,
            parameter_snapshot=parameters,ownership=after,auxiliary_nodes=after['auxiliary_nodes'],
            gateway_service_owners=services,remote_dependency=health,recorded_ns=time.time_ns())
        c.save(out/'result.json',result)
    except Exception as exc:
        result.update(error=repr(exc),recorded_ns=time.time_ns());c.save(out/'failure.json',result);raise


if __name__=='__main__':main()
