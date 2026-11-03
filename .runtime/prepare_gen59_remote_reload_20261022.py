"""Read current idle/remote readiness; --stop-remote stops only the exact old remote.

A fully validated, immutable install plan is mandatory even for the read-only
probe. No source install, parameter value write, mode switch, or motion occurs.
"""
import os
from pathlib import Path
import time
import xmlrpc.client
import gen59_remote_reload_common_20261022 as c


def main():
    p=c.parser(__doc__)
    p.add_argument('--stop-remote',action='store_true')
    p.add_argument('--install-plan',type=Path,required=True)
    p.add_argument('--install-plan-sha256',required=True)
    p.add_argument('--installer-sha256',required=True)
    args=p.parse_args(); ctx=c.context(args,__file__); out=args.output.resolve()
    c.storage(out); c.require(not out.exists(),'single-use output already exists')
    plan_path=args.install_plan.resolve(strict=True)
    c.require(c.O in plan_path.parents and c.sha(plan_path)==args.install_plan_sha256,'install plan changed')
    planned=c.read(plan_path); installer_path=c.W/'.runtime/install_and_seal_gen59_20261022.py'
    installer=c.module(installer_path,args.installer_sha256,'gen59_file_only_validator')
    c.require(planned['kind']=='GEN59_file_only_install_plan' and planned['helper_sha256']==args.installer_sha256,
              'install plan helper differs')
    validation=installer.validate(*map(Path,(planned['manifest'],planned['full'],planned['review'])))
    c.require(validation==planned['validation'],'full offline validation changed')
    ctx['pins'].update(validation['evidence_sha256'])
    ctx['pins'].update({str(plan_path):args.install_plan_sha256,str(installer_path):args.installer_sha256})
    baseline=c.read(c.BASE)['source_sha256']; c.recheck(ctx,baseline)
    old=ctx['legacy']; proof=c.read(old.ROSOUT_PROOF)
    c.require(c.sha(old.ROSOUT_PROOF)==old.ROSOUT_PROOF_SHA and proof['passed']
              and proof['live_ros_contacted'] is False and proof['remaining_after_seven']==[old.CACHED_KEYS[-1]+'/'],
              'bounded ROS cached-key cleanup proof changed')
    ctx['pins'][str(old.ROSOUT_PROOF)]=old.ROSOUT_PROOF_SHA
    ctx['pins'].update(proof['source_sha256'])
    for name in old.CACHED_KEY_SOURCE_FILES:
        ctx['pins'][str(c.W/name)]=baseline[name]
    out.mkdir(); os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    result=dict(passed=False,kind='GEN59_remote_reload_readiness',stop_requested=args.stop_remote,
        remote_stopped=False,motion_commands_sent=False,mode_changes_sent=False,temperature_queried=False,
        torque_disable_commands_sent=False,helper_path=str(Path(__file__).resolve()),helper_sha256=c.sha(__file__),
        common_path=str(Path(c.__file__).resolve()),common_sha256=c.sha(c.__file__),
        release_sha256=c.BASE_SHA,deployment_sha256=c.PREVIOUS_SHA,
        full_regression_path=planned['full'],full_regression_sha256=c.sha(planned['full']),
        independent_review_path=planned['review'],independent_review_sha256=c.sha(planned['review']),
        candidate_source_sha256=validation['source_sha256'],input_sha256=ctx['pins'],shutdown_requests=[],unsubscribe_requests=[])
    try:
        import rosgraph
        import rospy
        rospy.init_node('gen59_remote_prepare',anonymous=True,disable_rosout=True)
        master=rosgraph.Master(rospy.get_name())
        before=c.graph(ctx,rospy,master,ctx['nodes'][c.REMOTE])
        initial=c.sample_idle(ctx,rospy,master)
        binding_ref=ctx['previous']['remote_dependency']
        binding_path=Path(binding_ref['startup_binding'])
        binding=ctx['checker'].load_binding(binding_path,binding_ref['startup_binding_sha256'])
        c.require(binding['parameters']==ctx['config']['remote_parameters'],'old construction config differs')
        health_out=out/'remote_readiness'; health_out.mkdir()
        health=ctx['checker'].evaluate(binding,health_out); c.save(health_out/'result.json',health)
        c.require(health['passed'] and health['dependency_ready'],'current remote dependency unavailable')
        result['remote_dependency_before']=dict(startup_binding=str(binding_path),startup_binding_sha256=c.sha(binding_path),
            readiness=str(health_out/'result.json'),readiness_sha256=c.sha(health_out/'result.json'))
        ctx['pins'][str(binding_path)]=c.sha(binding_path)
        old_uri=master.lookupNode(c.REMOTE)
        values=c.fingerprints(ctx,master)
        result.update(initial,ownership_before=before,node_identities=ctx['nodes'],preserved_node_identities=ctx['preserved'],
            auxiliary_nodes=before['auxiliary_nodes'],old_remote_identity=ctx['nodes'][c.REMOTE],old_remote_uri=old_uri,
            core_service=before['core_service'],master_process=before['master_process'],simulation_service=before['simulation_service'],
            parameter_values_before=values,parameter_value_comparison_excludes=['/camera/runtime_profile/stationary_temporal'])
        c.recheck(ctx,baseline)
        c.require(c.graph(ctx,rospy,master,ctx['nodes'][c.REMOTE],auxiliaries=before['auxiliary_nodes'])==before,
                  'ownership changed during preflight')
        if not args.stop_remote:
            result.update(passed=True,recorded_ns=time.time_ns())
            c.save(out/'result.json',result); return
        c.save(out/'stop_plan.json',result)
        result['idle_before_stop']=c.sample_idle(ctx,rospy,master,initial['selection'],initial['epoch_ns'])
        c.require(c.identity(ctx,master,rospy.get_name(),c.REMOTE)==ctx['nodes'][c.REMOTE]
                  and master.lookupNode(c.REMOTE)==old_uri,'old remote changed at stop commit')
        c.recheck(ctx,baseline)
        request=dict(name=c.REMOTE,uri=old_uri,requested_ns=time.time_ns(),attempted=True)
        result['shutdown_requests'].append(request); c.save(out/'shutdown_intent.json',request)
        try:
            with xmlrpc.client.ServerProxy(old_uri,transport=ctx['old'].Transport()) as rpc:
                code,message,_=rpc.shutdown(rospy.get_name(),'load fully reviewed GEN59 observation repair; other 16 nodes preserved')
            request.update(return_code=code,message=message)
        except Exception as exc:
            request['rpc_error']=repr(exc)
        c.save(out/'shutdown_result.json',request)
        deadline=time.monotonic()+25.
        while time.monotonic()<deadline and not old.exited(ctx['nodes'][c.REMOTE]):
            c.recheck(ctx); time.sleep(.1)
        c.require(old.exited(ctx['nodes'][c.REMOTE]),'old remote did not exit; no duplicate launch')

        def absent_or_stale():
            c.recheck(ctx,baseline)
            c.require(old.exited(ctx['nodes'][c.REMOTE]) and not old.remote_processes(),'remote process reappeared')
            c.graph(ctx,rospy,master,auxiliaries=before['auxiliary_nodes'])
            system=master.getSystemState()
            c.require(not any(c.REMOTE in owners for rows in system for _,owners in rows),'remote topic/service still registered')
            try:
                uri=master.lookupNode(c.REMOTE)
            except rosgraph.MasterError:
                return False
            c.require(uri==old_uri,'remote URI changed')
            refusals=[]
            for _ in range(2):
                c.require(master.lookupNode(c.REMOTE)==old_uri,'old URI changed before refusal check')
                try:
                    with xmlrpc.client.ServerProxy(old_uri,transport=ctx['old'].Transport()) as rpc:
                        rpc.getPid(rospy.get_name())
                except ConnectionRefusedError as exc:
                    refusals.append(dict(error=type(exc).__name__,errno=exc.errno,checked_ns=time.time_ns()))
                else:
                    raise RuntimeError('old URI responded; cleanup prohibited')
                time.sleep(.05)
            result.setdefault('stale_refusal_checks',[]).append(refusals)
            c.require(old.exited(ctx['nodes'][c.REMOTE]) and not old.remote_processes()
                      and master.lookupNode(c.REMOTE)==old_uri,'remote appeared during refusal checks')
            return True

        stale_master=rosgraph.Master(c.REMOTE)
        for index,key in enumerate(old.CACHED_KEYS):
            if not absent_or_stale():
                break
            c.sample_idle(ctx,rospy,master,initial['selection'],initial['epoch_ns'])
            if not absent_or_stale():
                break
            row=dict(caller_id=c.REMOTE,caller_api=old_uri,key=key,requested_ns=time.time_ns(),attempted=True)
            c.save(out/('unsubscribe_%02d_intent.json'%index),row)
            c.recheck(ctx)
            c.require(master.lookupNode(c.REMOTE)==old_uri and old.exited(ctx['nodes'][c.REMOTE])
                      and not old.remote_processes() and master.getParam('/gui/joint_direct_mode') is True,
                      'cleanup commit ownership changed')
            row['rpc_return']=stale_master.unsubscribeParam(old_uri,key); row['returned_ns']=time.time_ns()
            result['unsubscribe_requests'].append(row); c.save(out/('unsubscribe_%02d_result.json'%index),row)
        c.require(not absent_or_stale(),'unknown stale registration remains; no expanded cleanup')
        result['parameter_values_after_cleanup']=c.fingerprints(ctx,master)
        c.same_parameters(values,result['parameter_values_after_cleanup'])
        final=c.sample_idle(ctx,rospy,master,initial['selection'],initial['epoch_ns'])
        after=c.graph(ctx,rospy,master,auxiliaries=before['auxiliary_nodes'])
        c.require(not absent_or_stale(),'remote registration reappeared')
        result.update(final,passed=True,kind='GEN59_remote_stopped_install_readiness',remote_stopped=True,
            old_remote_exited=True,remote_registration_absent=True,remote_processes_absent=True,ownership_after=after,
            parameter_configuration_unchanged=True)
        c.recheck(ctx,baseline); c.save(out/'result.json',result)
    except Exception as exc:
        result.update(passed=False,error=repr(exc),recorded_ns=time.time_ns())
        c.save(out/'failure.json',result)
        raise


if __name__=='__main__':
    main()
