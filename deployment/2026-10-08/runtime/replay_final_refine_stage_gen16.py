"""Replay only closed gen16 records. No ROS master, clients, publisher or actions."""
from pathlib import Path
from copy import deepcopy
from dataclasses import asdict
import hashlib, importlib.util, json, math, sys
import rosbag, rospy, tf2_ros, yaml

ROOT = Path(__file__).resolve().parents[1]
EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
FORMAL = EXTERNAL/'grasp_records/20260929T093457.667124Z_e74e4d2fb0884c9082d6c610d64bb948'
SCENE = EXTERNAL/'ros_log/feedforward_speed_grasp_20260929T093457Z'
OUT = SCENE/'offline_final_refine_stage'
assert Path('/media/zhuyupei/SanStand').is_mount(), 'external experiment storage required'
OUT.mkdir(exist_ok=True)
sys.path.insert(0, str(ROOT/'src/alicia_flexible_grasp_supervisor'))
from alicia_grasp_modes.observation_policy import observation_config

loaded_sources={}
def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    loaded_sources[name]=Path(path).read_bytes()
    exec(compile(loaded_sources[name],str(path),'exec'),module.__dict__)
    return module

source_path=ROOT/'src/alicia_flexible_grasp_supervisor/scripts/grasp_task_node.py'
before_path=ROOT/'.runtime/final_refine_stage_repair/before_grasp_task_node.py'
current = load('recorded_final_refine_new', source_path)
old = load('recorded_final_refine_old', before_path)
config_path=ROOT/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml'
selection=json.loads((FORMAL/'fresh_computation_phase.json').read_text())['selection']
frozen_config_path=OUT/'frozen_grasp_params.yaml'
if not frozen_config_path.exists(): frozen_config_path.write_bytes(config_path.read_bytes())
yaml_config=yaml.safe_load(frozen_config_path.read_text())
gcfg=observation_config(yaml_config['grasp'], selection)
# Store every requested default. There is no ROS parameter server connection.
parameters={}
def param(name, default=None):
    value=yaml_config
    supplied=True
    for part in name.strip('/').split('/'):
        if not isinstance(value,dict) or part not in value:
            value=default
            supplied=False
            break
        value=value[part]
    parameters[name]=dict(value=deepcopy(value),source='frozen_grasp_yaml' if supplied else 'production_default')
    return deepcopy(value)
rospy.get_param=param

def forbidden(*args, **kwargs):
    raise RuntimeError('live ROS operation forbidden during closed-record replay')
for name in ('init_node','ServiceProxy','wait_for_service','wait_for_message','Publisher','Subscriber'):
    setattr(rospy,name,forbidden)

wanted=['/tf','/tf_static','/perception/object','/grasp_6d/object_geometry','/grasp_6d/preview_plan_enriched','/grasp/near_field_phase','/grasp/state']
with rosbag.Bag(str(FORMAL/'keyframes.bag')) as bag:
    events=[(t.to_nsec(),topic,m) for topic,m,t in bag.read_messages(topics=wanted)]
contact=[(t,m) for t,topic,m in events if topic=='/grasp_6d/preview_plan_enriched' and m.diagnostic=='CONTACT_EXECUTION_PLAN']
assert len(contact)==2
bound_received,bound=contact[0]
preview_received,preview=contact[1]
phase=next(m for t,topic,m in events if topic=='/grasp/near_field_phase' and m.active and m.header.stamp.to_nsec()<preview.header.stamp.to_nsec() and m.header.stamp.to_nsec()>bound_received)
terminal=next(m for t,topic,m in events if topic=='/grasp/state' and m.message.startswith('FINAL_REFINE_TIMEOUT:'))
start=next(m for t,topic,m in events if topic=='/grasp/state' and m.message=='waiting for bounded final visual refinement')
minimum_ns=max(bound.header.stamp.to_nsec()+1,start.header.stamp.to_nsec()-int(gcfg['final_visual_refine_snapshot_slack_sec']*1e9),phase.header.stamp.to_nsec())
with rosbag.Bag(str(SCENE/'scene_diagnostic.bag')) as bag:
    scene_events=[(t.to_nsec(),topic,m) for topic,m,t in bag.read_messages(topics=['/tf','/tf_static','/perception/object'])]
scene_with_geometry=sorted(scene_events+[(t,topic,m) for t,topic,m in events if topic=='/grasp_6d/object_geometry'],key=lambda item:item[0])
normal_message=bound.object_geometry.support_normal_base
normal=current._normalize_vector3(tuple(getattr(normal_message,k) for k in 'xyz'),'recorded support normal')
q0=current._quaternion_xyzw(bound.poses[2]);q1=current._quaternion_xyzw(preview.poses[2])
j0=current._project_unit_to_plane(current._rotate_vector(q0,(0.,1.,0.)),normal,'recorded current jaw')
j1=current._project_unit_to_plane(current._rotate_vector(q1,(0.,1.,0.)),normal,'recorded observed jaw')
yaw=math.atan2(current._dot3(normal,current._cross3(j0,j1)),max(-1.,min(1.,current._dot3(j0,j1))))
a0=current._normalize_vector3(current._rotate_vector(q0,(0.,0.,1.)),'recorded current approach')
a1=current._normalize_vector3(current._rotate_vector(current._axis_angle_quaternion(normal,-yaw),current._rotate_vector(q1,(0.,0.,1.))),'recorded observed approach without yaw')
pose_metrics=dict(grasp_translation_m=math.sqrt(sum((getattr(preview.poses[2].position,k)-getattr(bound.poses[2].position,k))**2 for k in 'xyz')),yaw_deg=math.degrees(yaw),observed_roll_pitch_change_deg=math.degrees(math.acos(max(-1.,min(1.,current._dot3(a0,a1))))),registered_surface_translation_m=preview.refinement_translation_m,registered_surface_rotation_deg=preview.refinement_rotation_deg)
rows=[]
for evidence_name,evidence_events in [('formal_keyframes',events),('scene_tf_object_with_formal_geometry',scene_with_geometry)]:
    for delta_ns in (0,50_000_000,500_000_000,1_000_000_000,1_500_000_000):
        now_ns=preview_received+delta_ns
        if now_ns>=phase.deadline.to_nsec(): continue
        rospy.Time.now=staticmethod(lambda ns=now_ns: rospy.Time(ns//10**9,ns%10**9))
        buf=tf2_ros.Buffer(cache_time=rospy.Duration(300),debug=False)
        obj=geometry=None
        object_received=geometry_received=None
        for t,topic,m in evidence_events:
            if t>now_ns: break
            if topic in ('/tf','/tf_static'):
                for tr in m.transforms:
                    (buf.set_transform_static if topic=='/tf_static' else buf.set_transform)(tr,'recorded_accepted_rsp')
            elif topic=='/perception/object': obj=deepcopy(m);object_received=t
            elif topic=='/grasp_6d/object_geometry': geometry=deepcopy(m);geometry_received=t
        assert obj is not None and geometry is not None
        def node(module):
            n=module.GraspTaskNode.__new__(module.GraspTaskNode)
            n.active=True
            n.latest_grasp6d_preview_plan=deepcopy(preview)
            n.latest_obj=deepcopy(obj)
            n.latest_obj_time=obj.header.stamp
            n.latest_target_geometry=deepcopy(geometry)
            n.tf_buffer=buf
            # No occlusion exemption: retain the actual recorded detected target.
            n._bound_target_occlusion_allowed=False
            return n
        results={}
        n_new=node(current)
        for key,n,kw in [('old_helper',node(old),{}),('new_observation',n_new,{'preview_stage':'observation'}),('new_final_refinement',n_new,{'preview_stage':'final_refinement'})]:
            result,candidate=n._copy_near_field_preview_candidate(bound,minimum_ns,gcfg,**kw)
            results[key]=dict(asdict(result),candidate_plan_id=getattr(candidate,'plan_id',None))
        policy=current._final_refinement_registration_policy(gcfg)
        result,refined,metrics=current.build_bounded_final_visual_refinement(bound,preview,policy['maximum_translation_m'],math.radians(policy['maximum_yaw_deg']),math.radians(gcfg['final_visual_refine_max_roll_pitch_change_deg']))
        build=dict(asdict(result),metrics=metrics,refined_plan_id=getattr(refined,'plan_id',None))
        if refined is not None:
            build['refined_contact_evidence']=current.decode_contact_evidence(refined,required=True)
        tr=buf.lookup_transform_core('base_link','camera_link',rospy.Time(0))
        camera=[getattr(tr.transform.translation,k) for k in 'xyz']
        center=[getattr(preview.object_geometry.pose_base.position,k) for k in 'xyz']
        rows.append(dict(evidence_stream=evidence_name,now_ns=now_ns,remaining_original_deadline_sec=(phase.deadline.to_nsec()-now_ns)*1e-9,preview_source_age_sec=(now_ns-preview.header.stamp.to_nsec())*1e-9,object_source_ns=obj.header.stamp.to_nsec(),object_received_ns=object_received,geometry_source_ns=geometry.header.stamp.to_nsec(),geometry_received_ns=geometry_received,camera_tf_source_ns=tr.header.stamp.to_nsec(),camera_position_base_m=camera,preview_center_base_m=center,camera_target_distance_m=math.sqrt(sum((a-b)**2 for a,b in zip(camera,center))),results=results,bounded_refinement=build))
    
def ros_json(m):
    if hasattr(m,'__slots__'): return {s:ros_json(getattr(m,s)) for s in m.__slots__}
    if isinstance(m,(list,tuple)):return [ros_json(v) for v in m]
    if isinstance(m,bytes):return list(m)
    return m

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
report=dict(scope='closed recorded gen16 only; no current TF, no ROS master, no execution or simulated-success substitution',selection=selection,bound_plan_id=bound.plan_id,final_preview_plan_id=preview.plan_id,minimum_source_ns=minimum_ns,preview_received_ns=preview_received,phase_start_ns=phase.header.stamp.to_nsec(),phase_deadline_ns=phase.deadline.to_nsec(),first_preview_remaining_deadline_sec=(phase.deadline.to_nsec()-preview_received)*1e-9,original_failure=terminal.message,configuration=dict(source=str(config_path),frozen_copy=str(frozen_config_path),sha256=sha(frozen_config_path),application='grasp YAML plus production observation_config for recorded unknown/two_stage selection; not a complete archived runtime parameter dump',gcfg=gcfg,default_parameter_reads=parameters),source_sha256=dict(before=hashlib.sha256(loaded_sources['recorded_final_refine_old']).hexdigest(),after=hashlib.sha256(loaded_sources['recorded_final_refine_new']).hexdigest(),script=sha(Path(__file__))),pose_metrics=pose_metrics,input_paths=dict(formal=str(FORMAL/'keyframes.bag'),scene=str(SCENE/'scene_diagnostic.bag')),rows=rows,limits=['MoveIt sequence and digital-twin calls intentionally not executed; no recorded result for refined candidate exists.', 'Bag reception approximates task callback availability; five receipt-relative instants tested independently with both recorded TF/ObjectPose streams.', 'Replay does not claim a successful physical grasp.'],hardware_commands_sent=0,live_ros_calls=0)
(OUT/'replayed_before_grasp_task_node.py').write_bytes(loaded_sources['recorded_final_refine_old'])
(OUT/'replayed_after_grasp_task_node.py').write_bytes(loaded_sources['recorded_final_refine_new'])
def finite_json(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:finite_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [finite_json(v) for v in value]
    return value
report=finite_json(report)
(OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,default=str)+'\n')
(OUT/'recorded_plans.json').write_text(json.dumps(dict(bound=ros_json(bound),final_preview=ros_json(preview)),ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(report=str(OUT/'report.json'),first_preview_remaining_deadline_sec=report['first_preview_remaining_deadline_sec'],pose_metrics=pose_metrics,comparison_count=len(rows),old_rejections=sorted(set(r['results']['old_helper']['code'] for r in rows)),observation_rejections=sorted(set(r['results']['new_observation']['code'] for r in rows)),final_helper_passes=sum(r['results']['new_final_refinement']['ok'] for r in rows),bounded_rejections=sorted(set(r['bounded_refinement']['code'] for r in rows))),ensure_ascii=False,indent=2))
