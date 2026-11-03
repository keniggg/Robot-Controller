"""Install exact fully tested GEN60 closure, then one original-parent gateway respawn.

Requires --execute and reviewed immutable readiness. Never launches a node,
changes mode, resets task/remote, or invokes any motion/enable/temperature API.
"""
import os
from pathlib import Path
import time
import xmlrpc.client
import gen60_gateway_reload_common_20261022 as c


def atomic_install_file(relative,expected_before,expected_after,backup_dir,journal):
    c.require(relative in c.contract.CHANGED,'install path outside frozen five-file closure')
    source=c.contract.C/relative;destination=c.W/relative
    c.require(source.is_file() and not source.is_symlink() and c.sha(source)==expected_after,'candidate bytes changed')
    c.require(destination.parent.resolve()==destination.parent and not destination.is_symlink(),'unsafe source destination')
    existed=destination.exists()
    c.require(existed==(expected_before is not None),'install destination presence changed')
    if existed:c.require(destination.is_file() and c.sha(destination)==expected_before,'baseline source changed')
    raw=source.read_bytes();original=destination.read_bytes() if existed else None
    backup=backup_dir/relative;backup.parent.mkdir(parents=True,exist_ok=True)
    if existed:
        with backup.open('xb') as stream:stream.write(original);stream.flush();os.fsync(stream.fileno())
        c.require(c.sha(backup)==expected_before,'backup readback failed')
    row=dict(path=relative,before_sha256=expected_before,after_sha256=expected_after,
        backup=str(backup) if existed else None,operation='atomic_replace' if existed else 'atomic_create')
    journal('install_intent',row)
    temporary=destination.with_name(destination.name+'.gen60-install-'+str(os.getpid()))
    try:
        with temporary.open('xb') as stream:
            stream.write(raw);stream.flush();os.fsync(stream.fileno())
        os.chmod(temporary,destination.stat().st_mode&0o777 if existed else source.stat().st_mode&0o777)
        c.require(c.sha(source)==expected_after and c.sha(temporary)==expected_after
            and ((destination.exists() and c.sha(destination)==expected_before) if existed else not destination.exists()),
            'source/destination raced before atomic install')
        os.replace(temporary,destination)
        descriptor=os.open(destination.parent,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(descriptor)
        finally:os.close(descriptor)
        c.require(c.sha(destination)==expected_after,'installed source readback failed')
        journal('install_complete',row)
        return row
    finally:
        if temporary.exists():temporary.unlink()


def quarantine_target_cache(out,journal):
    rows=[]
    for status in c.target_caches():
        if not status['usable']:continue
        path=Path(status['path']);target=out/'target_cache_backup'/path.name
        target.parent.mkdir(exist_ok=True)
        st=path.stat()
        c.require((st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns)==tuple(status[k] for k in ('device','inode','size','mtime_ns'))
            and c.sha(path)==status['sha256'],'target cache changed before backup')
        raw=path.read_bytes()
        with target.open('xb') as stream:stream.write(raw);stream.flush();os.fsync(stream.fileno())
        c.require(c.sha(target)==status['sha256'] and c.sha(path)==status['sha256'],'cache backup readback differs')
        journal('target_cache_remove_intent',dict(status,backup=str(target)))
        st=path.stat();c.require((st.st_dev,st.st_ino,st.st_size,st.st_mtime_ns)==tuple(status[k] for k in ('device','inode','size','mtime_ns')),'cache raced')
        path.unlink();rows.append(dict(status,backup=str(target),removed=True));journal('target_cache_removed',rows[-1])
    c.require(not any(r['usable'] for r in c.target_caches()),'usable target cache remains')
    return rows


def main():
    p=c.parser(__doc__,evidence=True)
    p.add_argument('--preflight',type=Path,required=True);p.add_argument('--preflight-sha256',required=True)
    p.add_argument('--execute',action='store_true');args=p.parse_args()
    c.require(args.execute,'explicit --execute required; authoring alone never installs/reloads')
    ctx=c.context(args,__file__);c.reviewed_helpers(ctx,args);checked=c.frozen_evidence(ctx,args)
    pre=c.bind(ctx,args.preflight,args.preflight_sha256)
    c.require(pre['passed'] and pre['kind']=='GEN60_gateway_idle_install_readiness'
        and pre['node_identities']==ctx['nodes'] and pre['preserved_node_identities']==ctx['preserved']
        and pre['helper_sha256']==c.helper_map() and pre['deployment_sha256']==c.PREVIOUS_SHA
        and pre['release_sha256']==c.BASE_SHA,'preflight binding differs')
    c.require(pre['frozen_evidence']=={key:dict(path=str(getattr(args,key).resolve()),sha256=getattr(args,key+'_sha256'))
        for key in ('manifest','full','review')} and pre['helper_review_sha256']==args.helper_review_sha256,'preflight tested/reviewed evidence differs')
    c.require(0<=time.time_ns()-pre['recorded_ns']<=300_000_000_000,'preflight expired; obtain new read-only preflight')
    out=args.output.resolve();c.storage(out)
    c.require(out==c.O/'runtime_reload_r1' and not out.exists() and not c.RELEASE.exists(),'one new fixed transition/release required')
    out.mkdir();os.environ['ROS_LOG_DIR']=str(out/'observer_ros')
    sequence=[0]
    def journal(stage,detail):
        sequence[0]+=1;c.save(out/('%03d_%s.json'%(sequence[0],stage)),dict(stage=stage,recorded_ns=time.time_ns(),detail=detail))
    result=dict(kind='GEN60_gateway_source_install_and_respawn',passed=False,acceptance_passed=False,
        generation='GEN60',source_installed=False,gateway_reloaded=False,shutdown_requests=[],
        installed_paths=[],partial_source_install=False,automatic_retry_allowed=False,
        runtime_deployment_state='old_process_retained_before_install',
        preflight=str(args.preflight.resolve()),preflight_sha256=args.preflight_sha256,
        helper_sha256=c.helper_map(),helper_review=str(args.helper_review.resolve()),helper_review_sha256=args.helper_review_sha256,
        input_sha256=ctx['pins'],motion_commands_sent=False,mode_changes_sent=False,
        temperature_queried=False,torque_disable_commands_sent=False,physical_grasp_verified=False)
    try:
        import rospy
        import rosgraph
        rospy.init_node('gen60_gateway_install_reload',anonymous=True,disable_rosout=True)
        master=rosgraph.Master(rospy.get_name());old=ctx['nodes'][c.GATEWAY];ctx['expected_gateway']=old
        ctx['idle_reference']=dict(names=pre['names'],positions=pre['positions'])
        before=c.graph(ctx,rospy,master,auxiliaries=pre['auxiliary_nodes'])
        c.same_parameters(pre['parameter_snapshot'],c.parameters(master,out,'initial'),out,'initial')
        proof=c.source_load_proof(ctx,old,checked['baseline_source_sha256'],pre['gateway_original_parent'],allow_usable_cache=True)
        idle=c.sample_idle(ctx,rospy,master,pre['selection'],pre['epoch_ns'])
        c.service_owners(ctx,master);result['remote_dependency_before']=c.fresh_remote_health(ctx,out/'remote_readiness_before')
        c.recheck(ctx,checked['baseline_source_sha256']);c.graph(ctx,rospy,master,auxiliaries=pre['auxiliary_nodes'],graph_wait=0.)
        c.same_parameters(pre['parameter_snapshot'],c.parameters(master,out,'before_install'),out,'before_install')
        # The old gateway eagerly imported this module before service creation.
        # Write production last, then shut down once; stopping first would race
        # the original parent's two-second automatic respawn against installation.
        order=sorted(c.contract.CHANGED-{c.contract.SELECTOR})+[c.contract.SELECTOR]
        result['install_rows']=[]
        for relative in order:
            result['install_rows'].append(atomic_install_file(relative,checked['baseline_source_sha256'].get(relative),
                checked['source_sha256'][relative],out/'source_backup',journal))
            result['installed_paths'].append(relative)
            result['partial_source_install']=len(result['installed_paths'])<len(order)
            result['runtime_deployment_state']='source_changed_old_process_not_reloaded'
        result['target_cache_quarantine']=quarantine_target_cache(out,journal)
        installed=c.frozen_evidence(ctx,args,installed=True)
        result['source_installed']=True
        proof=c.source_load_proof(ctx,old,installed['source_sha256'],pre['gateway_original_parent'])
        result['installed_old_process_proof']=dict(proof,source_on_disk_only_old_process_not_reloaded=True)
        final_before=c.sample_idle(ctx,rospy,master,pre['selection'],pre['epoch_ns'])
        c.graph(ctx,rospy,master,auxiliaries=pre['auxiliary_nodes'],graph_wait=0.)
        c.same_parameters(pre['parameter_snapshot'],c.parameters(master,out,'before_shutdown'),out,'before_shutdown');c.service_owners(ctx,master)
        c.recheck(ctx,installed['source_sha256'])
        release=dict(generation='GEN60',passed=True,reviewed=True,installed=True,full_regression_passed=True,
            installed_bytes_exact_tested_candidate=True,baseline_release=str(c.BASE),baseline_release_sha256=c.BASE_SHA,
            previous_deployment=str(c.PREVIOUS),previous_deployment_sha256=c.PREVIOUS_SHA,
            source_sha256=installed['source_sha256'],source_count=1097,test_scope_files=352,test_summary=installed['test_summary'],
            candidate_manifest=str(args.manifest.resolve()),candidate_manifest_sha256=args.manifest_sha256,
            actual_regression=str(args.full.resolve()),actual_regression_sha256=args.full_sha256,
            review=str(args.review.resolve()),review_sha256=args.review_sha256,
            evidence_sha256=installed['evidence_sha256'],install_rows=result['install_rows'],
            runtime_deployment_accepted=False,physical_grasp_verified=False,recorded_ns=time.time_ns())
        c.save(c.RELEASE,release);result.update(source_manifest=str(c.RELEASE),source_manifest_sha256=c.sha(c.RELEASE))
        uri=master.lookupNode(c.GATEWAY);endpoint=xmlrpc.client.ServerProxy(uri,transport=ctx['legacy'].Transport())
        code,_,pid=endpoint.getPid(rospy.get_name())
        c.require(code==1 and ctx['v'].process(pid)==old,'exact gateway changed immediately before shutdown')
        request=dict(name=c.GATEWAY,uri=uri,identity=old,attempted=True,requested_ns=time.time_ns())
        result['shutdown_requests'].append(request);journal('single_gateway_shutdown_intent',request)
        result['runtime_deployment_state']='single_shutdown_requested_replacement_not_accepted'
        try:
            code,message,_=endpoint.shutdown(rospy.get_name(),'load fully reviewed GEN60 held endpoint selector while manual and idle')
            c.require(code==1,'gateway shutdown rejected: '+str(message));request.update(code=code,message=message)
        except (OSError,xmlrpc.client.Error) as exc:
            request['transport_error']=repr(exc)
        journal('single_gateway_shutdown_reply',request)
        deadline=time.monotonic()+40.;gateway=None
        while time.monotonic()<deadline:
            c.require(ctx['v'].process(proof['parent_identity']['pid'])==proof['parent_identity'],'original parent changed')
            c.require({name:c.identity(ctx,master,rospy.get_name(),name) for name in ctx['preserved']}==ctx['preserved'], 'preserved16 changed')
            if ctx['legacy'].exited(old):
                try:
                    candidate=c.identity(ctx,master,rospy.get_name(),c.GATEWAY)
                    if candidate!=old and candidate['parent_pid']==old['parent_pid']:
                        gateway=candidate;break
                except (rosgraph.MasterError,OSError,xmlrpc.client.Error):pass
            time.sleep(.1)
        c.require(gateway is not None and ctx['legacy'].exited(old),'original-parent replacement unavailable; no retry/duplicate launch')
        result.update(observed_new_gateway_identity=gateway,old_gateway_exited=True,
            runtime_deployment_state='replacement_observed_post_reload_checks_pending')
        journal('replacement_observed',dict(new_gateway_identity=gateway,old_gateway_exited=True))
        ctx['expected_gateway']=gateway
        for service in c.read(c.DESIGN)['constructor_review']['registered_service_names']:
            rospy.wait_for_service(service,timeout=8.)
        after=c.graph(ctx,rospy,master,gateway=gateway)
        c.require(not after['auxiliary_nodes'],'new idle gateway unexpectedly initialized a planner alias')
        services=c.service_owners(ctx,master)
        load=c.source_load_proof(ctx,gateway,installed['source_sha256'],proof['parent_identity'])
        c.same_gateway_load_contract(pre['gateway_source_load_proof'],load)
        c.same_parameters(pre['parameter_snapshot'],c.parameters(master,out,'after_respawn'),out,'after_respawn')
        health=c.fresh_remote_health(ctx,out/'remote_readiness_after')
        after_idle=c.sample_idle(ctx,rospy,master,pre['selection'],pre['epoch_ns'])
        c.require(c.graph(ctx,rospy,master,gateway=gateway,auxiliaries=after['auxiliary_nodes'])==after,'replacement ownership changed')
        c.same_parameters(pre['parameter_snapshot'],c.parameters(master,out,'final'),out,'final');c.recheck(ctx,installed['source_sha256'])
        result.update(after_idle,passed=True,gateway_reloaded=True,old_gateway_exited=True,
            runtime_deployment_state='new_gateway_loaded_independent_acceptance_pending',
            before=ctx['nodes'],full_runtime_nodes=after['node_identities'],new_gateway_identity=gateway,
            preserved_before=ctx['preserved'],preserved_after=ctx['preserved'],ownership=after,
            gateway_original_parent=proof['parent_identity'],gateway_source_load_proof=load,
            gateway_service_owners=services,parameter_snapshot=pre['parameter_snapshot'],remote_dependency=health,
            recorded_ns=time.time_ns())
        c.save(out/'result.json',result)
    except BaseException as exc:
        # Derive the actual on-disk subset even if an exception occurred between
        # rename and appending the successful row. Never claim no installation
        # merely because the complete-closure flag is still false.
        actual=[p for p in c.contract.CHANGED if (c.W/p).is_file() and
                c.sha(c.W/p)==checked['source_sha256'][p]]
        result.update(error=repr(exc),recorded_ns=time.time_ns(),installed_paths=sorted(actual),
            partial_source_install=bool(actual) and len(actual)<len(c.contract.CHANGED),
            source_installed=len(actual)==len(c.contract.CHANGED),automatic_retry_allowed=False)
        if actual and not result['shutdown_requests']:
            result['runtime_deployment_state']='source_changed_old_process_not_reloaded'
        c.save(out/'failure.json',result);raise


if __name__=='__main__':main()
