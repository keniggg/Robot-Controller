"""Actual remote node without ROS I/O, seeded only from historical PRE messages."""
from copy import deepcopy
import hashlib
import importlib.util
import json
import threading
from pathlib import Path

from std_msgs.msg import String
from sensor_msgs.msg import Image
from alicia_flexible_grasp_supervisor.msg import ObjectPose, Grasp6DPlan
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest
from alicia_grasp_modes.observe_return_session import parse_session

class LocalPublisher:
    def __init__(self, callback=None):
        self.messages=[]
        self.callback=callback
    def publish(self, message):
        self.messages.append(deepcopy(message))
        if self.callback is not None:
            self.callback(message)

def load_seed(directory):
    directory=Path(directory)
    manifest=json.loads((directory/'manifest.json').read_text())
    result={}
    classes={'pre_object':ObjectPose,'pre_mask':Image,'pre_status':String,
             'original_view_plan':Grasp6DPlan,'original_session':String}
    for row in manifest['rows']:
        raw=Path(row['file']).read_bytes()
        assert hashlib.sha256(raw).hexdigest()==row['sha256']
        value=classes[row['kind']]().deserialize(raw)
        assert value._md5sum==row['md5sum']
        result.setdefault(row['kind'],value)
    return result,manifest

def build_remote(candidate, producer, source_clock_ns, executing, seed_directory,
                 ack_delivery=None, parameters=None, lookup_transform=None,
                 initial_surface_directories=None):
    """Call only inside the harness's patched, advancing original source clock.

    The assigned stream generation/target epoch and observing session reproduce
    recorded pre-arrival state. No arrival receipt or source check is bypassed.
    """
    script=Path(candidate)/'src/alicia_grasp_modes/scripts/mode_aware_remote_grasp6d.py'
    spec=importlib.util.spec_from_file_location('gen14_recorded_remote',script)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    base=module.original
    seed,manifest=load_seed(seed_directory)
    node=module.ModeAwareRemoteGrasp6DNode.__new__(module.ModeAwareRemoteGrasp6DNode)
    node._initialize_mode_boundary()
    node._initialize_streaming_state(start_worker=False,source_clock=lambda:source_clock_ns()*1e-9)
    node._ros_source_clock_ns=source_clock_ns
    node._object_lock=threading.RLock();node._geometry_state_lock=threading.RLock()
    node.enabled=True;node.robot_execution_active=False
    configuration=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5/perception_execution_configuration.json')
    original_grasp=json.loads(configuration.read_text())['parameters']['/grasp']
    node.grasp_config=deepcopy(original_grasp)
    node.near_field_direct_timeout_sec=float(original_grasp.get('near_field_replan_timeout_sec',60.))
    node.near_field_strategy=str(original_grasp.get('near_field_strategy','legacy_gated')).strip().lower()
    node.frames=base.SynchronizedRgbdBuffer(source_clock_ns=source_clock_ns)
    node.planning_snapshot_max_age_sec=.35
    node.planning_snapshot_max_inference_latency_sec=3.5
    geometry_audit=None
    if parameters is not None:
        from gen14_remote_fusion_fixture_20261013 import configure_remote_geometry
        assert lookup_transform is not None
        geometry_audit=configure_remote_geometry(node,base,parameters,lookup_transform)
    node.latest_joint_state=None;node.latest_object=None;node.latest_object_time=None
    node.bridge=producer.bridge
    for name in ('plan_pub','rich_plan_pub','preview_plan_pub','preview_rich_plan_pub',
                 'status_pub','pipeline_metrics_pub','geometry_pub'):
        setattr(node,name,LocalPublisher())
    assert node.reset_mode_cb(None).success
    assert node.start_streaming()
    # Exact recorded identity, not an increment chosen to make a new frame pass.
    track=executing['target_track_id']
    generation_text,epoch_text=track.split('-t')
    assert generation_text.startswith('g') and int(epoch_text)==executing['target_epoch']
    node._target_identity_generation=int(generation_text[1:])
    node.target_instance_epoch=executing['target_epoch']
    assert node._current_stream_target_identity().track_id==track
    for kind in sorted(('pre_object','pre_mask','pre_status'),key=manifest['original_intertopic_receive_offsets_ns'].get):
        message=deepcopy(seed[kind])
        if kind=='pre_status':node._silhouette_source_status_cb(message)
        elif kind=='pre_object':node.object_cb(message)
        else:node.mask_cb(message)
    assert node.latest_object.detected and node.latest_object.header.stamp.to_nsec()==manifest['source_ns']
    assert node._current_stream_target_identity().track_id==track
    session=parse_session(json.loads(seed['original_session'].data),
                          executing['selection'],executing['stamp_ns'])
    assert session['session_id']==executing['session_id']
    assert session['phase']=='observing'
    plan=seed['original_view_plan']
    assert committed_plan_digest(plan)==executing['plan_sha256']
    node._observe_return_prefix=deepcopy(plan)
    node._observe_return_origin_prefix=deepcopy(plan)
    node._observe_return_session=deepcopy(session)
    node._observe_return_phase='observing';node._observe_return_paused=True
    node._observe_return_saved=None
    initial_surface_audit=None
    if initial_surface_directories is not None:
        from gen14_remote_fusion_fixture_20261013 import freeze_initial_surface
        initial_surface_audit=freeze_initial_surface(node,base,initial_surface_directories)
    node._measurement_remote_ack_pub=LocalPublisher(ack_delivery)
    node._measurement_remote_epoch_ns=executing['control_epoch_ns']
    evidence=dict(seed_manifest=str(Path(seed_directory)/'manifest.json'),
        seed_manifest_sha256=hashlib.sha256((Path(seed_directory)/'manifest.json').read_bytes()).hexdigest(),
        remote_script_sha256=hashlib.sha256(script.read_bytes()).hexdigest(),
        base_script=str(base.__file__),base_script_sha256=hashlib.sha256(Path(base.__file__).read_bytes()).hexdigest(),
        target_identity_seed_from_original_executing_request=track,
        original_grasp_configuration_sha256=hashlib.sha256(configuration.read_bytes()).hexdigest(),
        near_field_direct_timeout_sec=node.near_field_direct_timeout_sec,
        historical_pre_source_ns=manifest['source_ns'],initial_session_seeded=True,
        arrival_qualification_seeded=False,recorded_intertopic_offsets_ns=manifest['original_intertopic_receive_offsets_ns'],
        geometry_configuration=geometry_audit,initial_surface_reconstruction=initial_surface_audit)
    return node,module,seed,evidence
