"""Inert recorded-input producer/remote/task callback integration.

Exact recorded pixels, TF, feedback, plan and request timestamps are used.
The initialized pre-motion checkpoint and parameter service are local adapters;
the integer virtual schedule is not evidence of original thread timing. No
appearance, geometry, ACK, task association or marker checks are replaced.
"""
import argparse
from collections import deque
from copy import deepcopy
from dataclasses import fields
import hashlib
import importlib.util
import io
import json
import os
import platform
from pathlib import Path
import resource
import sys
import threading
import time
import traceback
from types import ModuleType
from types import SimpleNamespace as NS

REAL_TIME = time.monotonic
W = Path(__file__).resolve().parents[1]
HELPER = W/'.runtime/replay_gen57_arrival_chain_20261020.py'
spec = importlib.util.spec_from_file_location('pure_replay_helpers', HELPER)
h = importlib.util.module_from_spec(spec); spec.loader.exec_module(h)
INPUT = h.DEFAULT_INPUT
GEOMETRY = INPUT.parent
RECORD = h.E/'grasp_records/20261021T020508.229978Z_585a1fd70e7e4886a808da45a7aba646'
STEP = 'startup'
R8 = GEOMETRY/'owner_handoff_r8'
DOWNSTREAM_HELPER = W/'.runtime/gen58_downstream_local_callbacks_20261021.py'


def wire(msg):
    stream = io.BytesIO(); msg.serialize(stream)
    return stream.getvalue()


def main(args):
    global STEP
    output = args.output.resolve()
    assert Path('/media/zhuyupei/SanStand').is_mount() and h.inside(output, h.E) and not output.exists()
    assert sys.flags.no_user_site
    output.mkdir(parents=True)
    h.OUTPUT = output; sys.dont_write_bytecode = True; sys.addaudithook(h.audit)
    (output/'runner_source.py').write_bytes(Path(__file__).read_bytes())
    (output/'helper_source.py').write_bytes(HELPER.read_bytes())
    started = REAL_TIME(); before = resource.getrusage(resource.RUSAGE_SELF)
    snapshot, pins, _ = h.freeze_sources(W)
    for package in ('alicia_grasp_modes', 'alicia_flexible_grasp_supervisor'):
        root = W/'src'/package
        extras = [root/'package.xml']
        extras += list((root/'scripts').glob('*.py')) + list((root/'tests').glob('*.py'))
        for path in extras:
            relative = str(path.relative_to(W)); blob = path.read_bytes()
            target = snapshot/relative; target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                target.write_bytes(blob)
            assert target.read_bytes() == blob
            pins[relative] = h.digest_bytes(blob)
    r8_pins=json.loads((R8/'source_manifest.json').read_text())['source_sha256']
    assert all(pins.get(path)==digest for path,digest in r8_pins.items()), 'frozen r8 source differs'
    assert all(h.sha(W/path) == digest for path, digest in pins.items())
    for name in ('tmp', 'ros', 'ros_home', 'cache'):
        (output/name).mkdir()
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:1', ROS_PACKAGE_PATH=str(snapshot/'src')+':/opt/ros/noetic/share',
        ROS_LOG_DIR=str(output/'ros'), ROS_HOME=str(output/'ros_home'), TMPDIR=str(output/'tmp'),
        XDG_CACHE_HOME=str(output/'cache'))
    sys.path[:0] = [str(snapshot/'src/alicia_grasp_modes/tests'),
        str(snapshot/'src/alicia_flexible_grasp_supervisor/tests'),
        str(W/'devel/lib/python3/dist-packages'), '/opt/ros/noetic/lib/python3/dist-packages']
    h.import_frozen(snapshot)
    h.save('source_manifest.json',dict(source_sha256=pins,snapshot_root=str(snapshot),
        runner_sha256=h.sha(__file__),helper_sha256=h.sha(HELPER)))
    np, table = h.np, h.table
    def elf_architecture(executable=sys.executable, bits='', linkage=''):
        with open(executable,'rb') as stream:header=stream.read(5)
        assert header[:4]==b'\x7fELF' and header[4] in (1,2)
        return ('32bit' if header[4]==1 else '64bit','ELF')
    platform.architecture=elf_architecture
    from _pytest.monkeypatch import MonkeyPatch
    import rosbag
    import rospy
    from std_msgs.msg import String, Header
    from sensor_msgs.msg import JointState
    from test_unknown_stream import rgb_interest_node, module as producer_module
    from test_unknown_pending_identity import consumer_for, deliver
    from test_task_modes import configured, module as task_module
    from test_remote_modes import Publisher
    from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
    from alicia_grasp_modes.observe_return_session import parse_session
    original_transform = producer_module.transform_matrix
    mp = MonkeyPatch()
    producer, legacy_clock, _, _, parameters = rgb_interest_node(mp)
    mp.setattr(producer_module, 'transform_matrix', original_transform)
    now = [h.REFERENCE+100_000_000]
    legacy_clock[0]=now[0]/1e9
    def at(ns): return rospy.Time(ns//10**9, ns%10**9)
    mp.setattr(rospy.Time, 'now', staticmethod(lambda: at(now[0])))
    mp.setattr(time, 'monotonic', lambda: (now[0]-h.REFERENCE)*1e-9+100.)
    mp.setattr(rospy, 'is_shutdown', lambda: False)
    for name in ('loginfo', 'logwarn', 'logwarn_throttle', 'logdebug', 'logerr'):
        mp.setattr(rospy, name, lambda *a, **kw: None)
    data_manifest = json.loads((INPUT/'manifest.json').read_text())
    rows = {row['source_stamp_ns']: row for row in data_manifest['rows']}
    inputs = {str(INPUT/'manifest.json'): h.sha(INPUT/'manifest.json')}
    for path in (INPUT/'tf_evidence.json', GEOMETRY/'measurement_messages.jsonl',
                 GEOMETRY/'sdk_messages.jsonl', RECORD/'perception_execution_configuration.json',
                 RECORD/'manifest.json', RECORD/'recorder_outcome.json'):
        inputs[str(path)] = h.sha(path)
    h.save('initial_input_manifest.json',dict(input_sha256=inputs))
    assert inputs[str(INPUT/'manifest.json')] == h.INPUT_MANIFEST_SHA
    def load(stamp):
        row = rows[stamp]; inputs[row['path']] = h.sha(row['path'])
        assert inputs[row['path']] == row['npz_sha256']
        with np.load(row['path'], allow_pickle=False) as archive:
            result = {key: h.immutable(archive[key]) for key in archive.files}
        result['T_base_camera'] = h.immutable(original_transform(h.namespace(row['exact_tf'])))
        result['depth'] = h.immutable(np.asarray(result['depth_raw'], dtype=np.float32)*float(result['depth_scale']))
        return result
    raw_messages = [json.loads(line) for line in (GEOMETRY/'measurement_messages.jsonl').read_text().splitlines()]
    executing = next(row['data'] for row in raw_messages if row['topic'].endswith('measurement_request') and row['data']['state']=='executing')
    arrived = next(row['data'] for row in raw_messages if row['topic'].endswith('measurement_request') and row['data']['state']=='requalifying')
    session_record = next(row['data'] for row in raw_messages if row['topic'].endswith('observe_return_session'))
    config_record = json.loads((RECORD/'perception_execution_configuration.json').read_text())['parameters']
    params = config_record['/unknown_perception']
    producer.config = table.Config(**{k:v for k,v in params.items() if k in {f.name for f in fields(table.Config)}})
    producer._rgb_interest_config = deepcopy(params)
    producer.generation = executing['selection']['generation']; producer.selection_ns = executing['selection']['stamp_ns']
    producer._rgb_interest_epoch = executing['control_epoch_ns']
    producer.anchor_source_stamp_ns = h.ANCHOR; producer.last_good_source_stamp_ns = h.REFERENCE
    producer.last_consumed_ns = h.REFERENCE; producer.last_shape = (489, 653)
    camera = deepcopy(rows[h.REFERENCE]['camera_geometry'])
    camera['runtime_profile'] = camera.pop('runtime_profile_geometry')
    parameters.update({'/camera': camera, '/unknown_perception': params,
        '/grasp_mode/selection': deepcopy(executing['selection']), '/grasp': deepcopy(config_record['/grasp']),
        '/gui/joint_direct_mode': False})
    context_provenance=None
    if args.context_parameters is not None:
        context_path=args.context_parameters.resolve(strict=True)
        assert args.context_parameters_sha256 and h.sha(context_path)==args.context_parameters_sha256
        context_values=json.loads(context_path.read_text())
        if 'parameters' in context_values:context_values=context_values['parameters']
        if '/handeye' not in context_values:
            handeye={key[len('/handeye/'):]:value for key,value in context_values.items()
                     if key.startswith('/handeye/')}
            if handeye:context_values['/handeye']=handeye
        assert isinstance(context_values,dict) and '/robot_description' in context_values and '/handeye' in context_values
        # Bind all recovered original launch values, then the later recorded
        # runtime snapshots for parameters that can change after launch.
        runtime_values=deepcopy(parameters)
        parameters.update(deepcopy(context_values))
        for key in ('/camera','/unknown_perception','/grasp_mode/selection','/grasp','/gui/joint_direct_mode'):
            parameters[key]=runtime_values[key]
        inputs[str(context_path)]=args.context_parameters_sha256
        context_provenance=dict(path=str(context_path),sha256=args.context_parameters_sha256)
    # Context bindings are frozen across every owner; metric base/camera
    # geometry always uses the exact recorded transforms.
    context_only = {'/robot_description': parameters['/robot_description'], '/handeye': parameters['/handeye']}
    tf_data = json.loads((INPUT/'tf_evidence.json').read_text())
    tool_camera = next(row for row in tf_data['static'] if row['header']['frame_id']=='tool0' and row['child_frame_id']=='camera_link')
    tf_calls = []
    def tf_lookup(base, child, stamp, timeout):
        ns = stamp.to_nsec(); tf_calls.append(dict(base=base, child=child, source_stamp_ns=ns))
        if base=='tool0' and child=='camera_link': return h.namespace(tool_camera)
        if base=='base_link' and child=='camera_link' and ns in rows: return h.namespace(rows[ns]['exact_tf'])
        raise ValueError('offline exact TF unavailable: '+str((base,child,ns)))
    producer.buffer = NS(lookup_transform=tf_lookup)
    frame_data = {stamp:load(stamp) for stamp in (h.ANCHOR,h.REFERENCE)+h.ARRIVALS+(h.STATION,)}
    def image_pair(stamp):
        data=frame_data[stamp]; row=rows[stamp]
        pair=(producer.bridge.cv2_to_imgmsg(data['color'].copy(), encoding='bgr8'),
              producer.bridge.cv2_to_imgmsg(data['depth_raw'].copy(), encoding=row['depth_encoding']))
        for msg, header in zip(pair,(row['color_header'],row['depth_header'])):
            msg.header.stamp=at(stamp);msg.header.frame_id=header['frame_id'];msg.header.seq=header['seq']
        return pair
    # Restore the measured raw anchor checkpoint, then recompute the fresh
    # pre-motion reference with the candidate production algorithm.
    STEP='initial_geometry'
    a=frame_data[h.ANCHOR]
    seg=table.segment_with_support_fallback(a['depth'],a['K'],a['T_base_camera'],producer.config)
    tracker=table.TargetTracker(producer.config);anchor,reason=tracker.choose(seg,a['depth'].shape)
    assert anchor is not None,reason
    plane=seg.plane_base.copy();producer.tracker=tracker
    ref=frame_data[h.REFERENCE]
    seg=table.segment_with_support_fallback(ref['depth'],ref['K'],ref['T_base_camera'],producer.config,
        anchor,anchor,plane,prefer_current_support=True)
    matches=table._matching_candidates(seg.candidates,anchor,anchor,producer.config);assert len(matches)==1
    raw=matches[0][1];producer.tracker.last=raw
    refined=h.refine_candidate(ref['color'],ref['depth'],ref['K'],ref['T_base_camera'],raw,producer.config,
                               rgb_valid=ref['rgb_valid'])
    joint_rows=[]
    for line in (GEOMETRY/'sdk_messages.jsonl').read_text().splitlines():
        row=json.loads(line)
        if row['topic']=='/alicia_d/accepted_joint_states':joint_rows.append(row)
    def joint_message(row):
        data=row['data'];msg=JointState();msg.header.stamp=at(data['header']['stamp']['ns'])
        msg.header.seq=data['header']['seq'];msg.header.frame_id=data['header']['frame_id']
        for name in ('name','position','velocity','effort'):setattr(msg,name,data[name])
        return msg
    delivered_joints=set();remote=None;task=None
    def feed_joints(cutoff, *, lower=None):
        for row in joint_rows:
            ns=row['data']['header']['stamp']['ns']
            if (ns in delivered_joints or ns>cutoff or row['bag_receipt_ns']>cutoff
                    or lower is not None and ns<lower):continue
            msg=joint_message(row);producer._rgb_interest_joint_cb(msg)
            if remote is not None:remote._station_compute_accepted_cb(msg)
            if task is not None:task.accepted_joint_cb(msg)
            delivered_joints.add(ns)
    feed_joints(now[0],lower=h.REFERENCE-1_500_000_000)
    reference_pair=image_pair(h.REFERENCE)
    reference=producer._prepare_locked_rgb_hold_reference(reference_pair,refined,camera,
        tracker=producer.tracker,source_transform=ref['T_base_camera'])
    assert reference is not None,'real reference preparation rejected'
    h.save('reference_source_context.json',producer._rgb_interest_source_context(reference_pair[0].header.stamp))
    if args.expected_reference_source_id is not None:
        assert reference['source_id']==args.expected_reference_source_id,(
            'original source context digest mismatch',reference['source_id'],args.expected_reference_source_id)
    producer._locked_rgb_hold_reference=reference
    producer._publish_measurement_candidate(reference_pair,refined)
    producer.status('ready','tracked',reference_pair,refined)
    h.save('reference_state.json',dict(reference_mode='reconstructed',
        historical_pixels_used=True,reference_mask_sha256=reference['refined_mask_sha256'],
        reference_recomputed_mask_matches_historical=bool(np.array_equal(refined.mask,ref['recorded_refined_mask'])),
        context_parameters=context_only,context_uses_fixture_parameters=context_provenance is None,
        source_id=reference['source_id'],
        recovered_context_parameter_provenance=context_provenance,
        original_source_id_exactly_reproduced=(args.expected_reference_source_id is not None),
        candidate=h.candidate_summary(refined),original_anchor_checkpoint_restored=True))
    STEP='original_plan_read'
    from alicia_flexible_grasp_supervisor.msg import Grasp6DPlan
    plan_provenance=json.loads((R8/'original_plan_provenance.json').read_text())
    plan_bytes=(R8/'original_plan.bin').read_bytes()
    assert h.digest_bytes(plan_bytes)==plan_provenance['serialized_sha256']
    plan=Grasp6DPlan();plan.deserialize(plan_bytes)
    assert committed_plan_digest(plan)==executing['plan_sha256']
    assert plan.header.stamp.to_nsec()==executing['view_source_stamp_ns']
    (output/'original_plan.bin').write_bytes(plan_bytes)
    h.save('original_plan_provenance.json',dict(reused_r8_export=True,
        r8_file=str(R8/'original_plan.bin'),r8_provenance_sha256=h.sha(R8/'original_plan_provenance.json'),
        original_provenance=plan_provenance,bag_read_this_run=False))
    STEP='owner_initialization'
    remote=consumer_for(producer,legacy_clock,parameters)
    remote._ros_source_clock_ns=lambda:now[0]
    remote._target_identity_generation=1;remote.target_instance_epoch=executing['target_epoch']
    assert remote._current_stream_target_identity().track_id==executing['target_track_id']
    ready=dict(object=deepcopy(producer.pubs['object'].publish.call_args[0][0]),
        mask=deepcopy(producer.pubs['object_mask'].publish.call_args[0][0]),
        status=json.loads(producer.pubs['detector_status'].publish.call_args[0][0].data))
    deliver(remote,ready)
    assert remote._unknown_identity_reference is not None
    remote._observe_return_prefix=deepcopy(plan);remote._observe_return_origin_prefix=deepcopy(plan)
    remote._observe_return_phase='observing';remote._observe_return_paused=True;remote._observe_return_saved=object()
    remote._measurement_remote_epoch_ns=executing['control_epoch_ns']
    session=deepcopy(session_record)
    for key in ('stamp_ns','deadline_ns','origin_snapshot_stamp_ns','return_verified_stamp_ns','view_source_stamp_ns'):
        session[key]=int(session[key])
    remote._observe_return_session=deepcopy(session)
    with mp.context() as setup:
        task,_=configured.__wrapped__(setup)
    # The fixture's own params are temporary; restore the common recorded
    # parameter service for both reset barriers and every later callback.
    task._mode_selection=deepcopy(executing['selection']);task._mode_bound_selection=deepcopy(executing['selection'])
    task.active=True;task.stage=2;task._execution_authority_revoked=False
    task._bound_execution_plan=deepcopy(plan);task._bound_execution_plan_id=plan.plan_id
    task._bound_execution_plan_digest=task._execution_plan_digest(plan);task.latest_grasp6d_plan=deepcopy(plan)
    task._observe_return_session=deepcopy(session)
    task._observe_return_origin_proof=dict(epoch_ns=executing['control_epoch_ns'])
    task._following_evidence_lock=threading.RLock();task._following_epoch_ns=executing['control_epoch_ns']
    task._following_accepted_history=deque(maxlen=128);task._following_sdk_history=deque(maxlen=128)
    task.latest_obj=None;task.latest_obj_time=None
    task.near_field_phase_pub=Publisher()
    bus=[];producer_acks=[];remote_acks=[];frame_results=[];requests=[]
    def record(kind,msg):
        item=dict(kind=kind,virtual_receipt_ns=now[0],message_type=msg._type,
            wire_sha256=h.digest_bytes(wire(msg)))
        if isinstance(msg,String):item['data']=json.loads(msg.data)
        elif hasattr(msg,'header'):item['source_stamp_ns']=msg.header.stamp.to_nsec()
        bus.append(item)
    class Route:
        def __init__(self,name,callback=None):self.name=name;self.callback=callback;self.messages=[]
        def publish(self,msg):
            msg=deepcopy(msg);self.messages.append(msg);record(self.name,msg)
            if self.callback is not None:self.callback(msg)
    def producer_ack(msg):
        producer_acks.append(json.loads(msg.data));remote._measurement_remote_ack_cb(msg);task._observation_measurement_ack_cb(msg)
    def remote_ack(msg):
        remote_acks.append(json.loads(msg.data));task._observation_measurement_ack_cb(msg)
    producer._observation_measurement_ack_pub=Route('unknown_ack',producer_ack)
    remote._measurement_remote_ack_pub=Route('remote_ack',remote_ack)
    producer.pubs['object']=Route('object',lambda msg:(remote.object_cb(msg),task.obj_cb(msg)))
    producer.pubs['object_mask']=Route('mask',remote.mask_cb)
    producer.pubs['detector_status']=Route('status',lambda msg:(remote._silhouette_source_status_cb(msg),task._unknown_support_status_cb(msg)))
    for name in ('object_detected','raw_object_detected','object_pose_camera','object_pose_base','rgb_reference_status'):
        producer.pubs[name]=Route(name)
    def process_frame(stamp):
        row=rows[stamp]
        # The original detector status receipt bounds this synthetic event
        # schedule; compute execution itself uses no historical timing claim.
        now[0]=max(row['detector_status']['bag_receipt_ns'],row['feedback_after'][0])
        legacy_clock[0]=now[0]/1e9
        feed_joints(now[0])
        producer.latest=image_pair(stamp);producer.process(None)
        status=json.loads(producer.pubs['detector_status'].messages[-1].data)
        frame_results.append(dict(source_stamp_ns=stamp,virtual_now_ns=now[0],status=status,
            unknown_latest_ack=None if not producer_acks else producer_acks[-1],
            remote_latest_ack=None if not remote_acks else remote_acks[-1]))
        h.save(str(stamp)+'_owner_frame.json',frame_results[-1])
        assert stamp not in h.ARRIVALS[:2] or not [
            message for message in producer.pubs['object'].messages
            if message.header.stamp.to_nsec()==stamp], 'early candidate published before third confirmation'
        measurement=status.get('observation_measurement',{})
        print(json.dumps(dict(source=stamp,detail=status.get('detail'),
            measurement_reason=measurement.get('reason'),qualified=measurement.get('qualified'),
            source_stamps_ns=measurement.get('source_stamps_ns'))),flush=True)
    def request_transport(msg):
        request=json.loads(msg.data);requests.append(request)
        producer._observation_measurement_request_cb(msg);remote._measurement_remote_request_cb(msg)
        if request['state']=='requalifying':
            assert request==arrived,'task did not reproduce original requalifying contract'
            for stamp in h.ARRIVALS:process_frame(stamp)
            assert producer_acks[-1]['qualified'],('producer callback chain did not qualify',frame_results[-1])
            assert remote_acks[-1]['qualified'],('remote callback chain did not qualify',remote_acks[-1])
    task._observation_measurement_pub=Route('task_request',request_transport)
    STEP='executing_real_task_dual_ack'
    now[0]=executing['stamp_ns'];legacy_clock[0]=now[0]/1e9;feed_joints(now[0])
    produced=task._observation_measurement_begin(plan)
    assert produced==executing,'task did not reproduce exact historical executing request'
    h.save('executing_receipts.json',dict(request=produced,unknown=producer_acks[-1],remote=remote_acks[-1]))
    STEP='arrival_real_task_finish'
    now[0]=arrived['stamp_ns'];legacy_clock[0]=now[0]/1e9;feed_joints(now[0])
    task._last_motion_settle_sample=deepcopy(arrived['settle'])
    task._observation_measurement_finish(plan,parameters['/grasp'])
    assert task._observation_measurement_request['state']=='qualified'
    assert producer._observation_measurement is None
    marker=producer._locked_rgb_hold_reference['observation_arrival_retry']
    assert marker['qualified_source_stamps_ns']==list(h.ARRIVALS)
    h.save('qualified_handoff.json',dict(request=task._observation_measurement_request,
        unknown_ack=producer_acks[-1],remote_ack=remote_acks[-1],
        marker_source_stamps_ns=marker['qualified_source_stamps_ns'],
        actual_task_object_stamp_ns=task.latest_obj.header.stamp.to_nsec(),
        station_template_mask_sha256=marker['station_appearance'].mask_sha256,
        producer_marker_from_real_qualified_callback=True,task_actual_source_join=True))
    phases=task.near_field_phase_pub.messages
    h.save('task_planning_phase.json',dict(actual_phase_count=len(phases),
        phases=[dict(active=phase.active,phase_id=phase.phase_id,
            header_stamp_ns=phase.header.stamp.to_nsec(),deadline_ns=phase.deadline.to_nsec(),
            reference_source_stamp_ns=phase.reference_source_stamp.to_nsec(),
            reference_target_track_id=phase.reference_target_track_id,
            planning_stage=phase.planning_stage,serialized_sha256=h.digest_bytes(wire(phase))) for phase in phases],
        later_remote_registration_not_replayed=True))
    # Export actual callback publications and continue only local computation.
    downstream_spec=importlib.util.spec_from_file_location('gen58_downstream_local',DOWNSTREAM_HELPER)
    downstream_module=importlib.util.module_from_spec(downstream_spec)
    downstream_spec.loader.exec_module(downstream_module)
    (output/'downstream_helper_source.py').write_bytes(DOWNSTREAM_HELPER.read_bytes())
    downstream_result=downstream_module.run(locals(),h)
    h.save('downstream_result.json',downstream_result)
    STEP='fourth_frame_real_producer'
    process_frame(h.STATION)
    status=frame_results[-1]['status']
    assert status['state']=='ready',status
    assert producer.last_good_source_stamp_ns==h.STATION
    assert producer._locked_rgb_hold_reference['observation_arrival_retry'] is marker
    assert all(h.sha(snapshot/path)==digest for path,digest in pins.items())
    assert all(h.sha(path)==digest for path,digest in inputs.items())
    h.save('bus_messages.json',bus)
    h.save('tf_access.json',tf_calls)
    provenance={};all_modules=list(sys.modules.values())+[producer_module,task_module]
    seen=set()
    for module in all_modules:
        if not isinstance(module,ModuleType) or id(module) in seen:continue
        seen.add(id(module))
        for attr in ('module','original','base'):
            candidate=getattr(module,attr,None)
            if isinstance(candidate,ModuleType) and id(candidate) not in seen:all_modules.append(candidate)
        filename=getattr(module,'__file__',None)
        if not filename:continue
        path=Path(filename).resolve()
        if h.inside(path,snapshot):
            relative=str(path.relative_to(snapshot));assert pins.get(relative)==h.sha(path)
            provenance[str(path)]=dict(sha256=h.sha(path),module_name=module.__name__,source='frozen_snapshot')
        elif h.inside(path,W/'devel'):
            provenance[str(path)]=dict(sha256=h.sha(path),module_name=module.__name__,source='generated_ROS_message')
        elif h.inside(path,W/'src'):
            raise RuntimeError('project module loaded outside source snapshot: '+str(path))
    required=[snapshot/relative for relative in (h.NODE,
        'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py',
        'src/alicia_grasp_modes/scripts/mode_aware_grasp_task.py',
        'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py',
        'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py')]
    assert all(str(path) in provenance for path in required)
    h.save('module_provenance.json',provenance)
    after=resource.getrusage(resource.RUSAGE_SELF)
    result=dict(passed=True,stage=STEP,source_stamps_ns=list(h.ARRIVALS),fourth_source_ns=h.STATION,
        real_producer_geometry=True,real_producer_qualified_ack=True,real_remote_same_source_join=True,
        real_task_dual_ack_and_object_join=True,real_qualified_station_marker=True,fourth_frame_ready=True,
        source_inputs_sha256=inputs,python_boundary_attempts=h.BOUNDARY_ATTEMPTS,
        sdk_delivery_requires_original_receipt_not_after_virtual_now=True,
        first_two_sources_publish_no_metric_object=True,
        original_clocks_or_thread_order_reproduced=False,reference_mode='reconstructed',
        recovered_context_parameter_provenance=context_provenance,
        original_source_id_exactly_reproduced=(args.expected_reference_source_id is not None),
        real_hardware_or_network_used=False,physical_grasp_verified=False,
        wall_seconds=REAL_TIME()-started,user_cpu_seconds=after.ru_utime-before.ru_utime,
        system_cpu_seconds=after.ru_stime-before.ru_stime,max_rss_kib=after.ru_maxrss,
        limitations=['Pre-motion measured tracker checkpoint restored; initial three-frame acquisition is not replayed.',
            'Fresh candidate reference recomputed from original pixels; this is not the historically frozen mask.',
            'Integer virtual message schedule preserves original source/deadline fields but does not validate wall-time performance or original thread ordering.',
            ('Model and handeye parameter values are frozen context-only fixture values; recorded exact base-camera and tool-camera transforms supply all metric geometry.'
             if context_provenance is None else 'Original launch context restored from hash-bound parameters; dynamic runtime camera and perception come from recorded source snapshots.'),
            'No remote prediction, actual motion, contact planning, collision execution or physical grip is claimed.'])
    print(json.dumps(h.save('result.json',result)),flush=True)
    mp.undo()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--context-parameters',type=Path)
    parser.add_argument('--context-parameters-sha256')
    parser.add_argument('--expected-reference-source-id')
    arguments=parser.parse_args()
    try:main(arguments)
    except Exception as exc:
        if h.OUTPUT is not None and h.OUTPUT.exists() and hasattr(h,'np'):
            h.save('failure.json',dict(stage=STEP,error=repr(exc),traceback=traceback.format_exc(),
                boundary_attempts=h.BOUNDARY_ATTEMPTS,passed=False))
        raise
