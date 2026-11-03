"""Inert task initialization from the sealed attempt's pre-arrival state.

This helper does not replace task admission, measurement parsing, source
association, ACK waiting, or phase creation. The replay owns a single advancing
clock and the local topic delivery schedule; no live ROS node is constructed.
"""
import ast
from collections import deque
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import threading
from types import SimpleNamespace

from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.observe_return_session import parse_session
from alicia_flexible_grasp.robot.observation_path_guard import committed_plan_digest


CONFIGURATION = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/grasp_records/'
    '20261013T025604.344486Z_af3ecb9373264cc9b19a98143f7b5df5/'
    'perception_execution_configuration.json')


def build_task(task_module, *, plan, session, executing_request, settle,
               publish_request, publish_state, publish_phase, publish_near_field,
               condition=None, original_geometry=None):
    """Seed only already recorded history; qualified/task arrival stays empty.

    Before calling finish, the harness must expose the original selection via
    get_param/get_master, deliver actual producer ObjectPose through obj_cb,
    and forward actual participant ACKs to _observation_measurement_ack_cb.
    publish_request enqueues public producer/remote callbacks. A condition
    adapter may drive due events during the original wait(.02), while keeping
    real CPU time and that wait in the same monotonically advancing clock.
    """
    selection = parse_selection(executing_request['selection'])
    assert executing_request['state'] == 'executing'
    assert plan.plan_id == executing_request['view_plan_id']
    assert plan.target_track_id == executing_request['target_track_id']
    assert committed_plan_digest(plan) == executing_request['plan_sha256']
    session = parse_session(session, selection, executing_request['stamp_ns'])
    assert session['phase'] == 'observing' and session['version'] == 2
    for key in ('session_id', 'target_epoch', 'target_track_id', 'origin_plan_id',
                'view_plan_id', 'view_index', 'deadline_ns',
                'origin_snapshot_stamp_ns', 'view_source_stamp_ns'):
        assert session[key] == executing_request[key], key
    assert settle['epoch_ns'] == executing_request['control_epoch_ns']

    payload = CONFIGURATION.read_bytes()
    parameters = json.loads(payload)['parameters']
    config = task_module.execution_config(deepcopy(parameters['/grasp']), 'observe_return')
    # The recorded task parameter is 90 s. The producer-only v3 diagnostic
    # had used a stricter 60 s phase; do not mislabel the default as this
    # attempt's configured value or silently modify production parameters.
    assert config.get('unknown_observe_return_requalification_timeout_sec', 3.) == 3.
    assert config['near_field_replan_timeout_sec'] == 90.

    cls = task_module.ModeAwareGraspTaskNode
    node = cls.__new__(cls)
    node._initialize_mode_boundary()
    if condition is not None:
        node._observe_return_condition = condition
    node._mode_selection = deepcopy(selection)
    node._mode_bound_selection = deepcopy(selection)
    node._mode_ready = True
    node._mode_start_reserved = False
    node.active = True
    node.stage = task_module.original.GraspStages.MOVE_PREGRASP
    node._start_inflight = False
    node._execution_authority_revoked = False
    node._bound_target_occlusion_allowed = False
    node._bound_execution_plan = deepcopy(plan)
    node._bound_execution_plan_id = plan.plan_id
    node._bound_execution_plan_digest = committed_plan_digest(plan)
    node.latest_grasp6d_plan = deepcopy(plan)
    node.latest_grasp6d_preview_plan = None
    node._grasp6d_plan_lock = threading.RLock()
    node._observe_return_session = deepcopy(session)
    node._observe_return_active_view_plan = deepcopy(plan)
    node._observe_return_origin_plan = deepcopy(plan)
    node._observe_return_entry_phase = plan.diagnostic
    node._observation_measurement_request = deepcopy(executing_request)
    node._observation_measurement_acks.clear()
    node._last_motion_settle_sample = deepcopy(settle)
    # Exact inert constructor values only, never station registration/history.
    original_source = Path(task_module.original.__file__)
    original_text = original_source.read_text(); original_ast = ast.parse(original_text)
    inert_fields = ('latest_joint_state', 'latest_accepted_joint_state', '_following_fk_cache')
    inert_audit = []
    for name in inert_fields:
        matches = [row for row in ast.walk(original_ast) if isinstance(row, ast.Assign)
            and any(isinstance(target, ast.Attribute) and isinstance(target.value, ast.Name)
                    and target.value.id == 'self' and target.attr == name for target in row.targets)
            and isinstance(row.value, ast.Constant) and row.value.value is None]
        assert matches, name
        setattr(node, name, None)
        inert_audit.append(dict(field=name, value=None, original_assignment_lines=[row.lineno for row in matches]))
    node._following_evidence_lock = threading.RLock()
    node._following_epoch_ns = executing_request['control_epoch_ns']
    node._following_accepted_history = deque(maxlen=128)
    node._following_sdk_history = deque(maxlen=128)
    node.latest_obj = node.latest_obj_time = None
    node.latest_visual_obj = node.latest_visual_obj_time = None
    node.latest_target_geometry = deepcopy(
        plan.object_geometry if original_geometry is None else original_geometry)
    node._near_field_active = False
    node._near_field_phase_id = 0
    node._published_near_field_phase = None
    node._near_field_phase_started_sec = 0.
    node._near_field_phase_deadline_sec = 0.
    node._near_field_reference_center_base = None
    node.pub = SimpleNamespace(publish=publish_state)
    node.near_field_pub = SimpleNamespace(publish=publish_near_field)
    node.near_field_phase_pub = SimpleNamespace(publish=publish_phase)
    node._observation_measurement_pub = SimpleNamespace(publish=publish_request)

    def forbid_motion(*args, **kwargs):
        raise AssertionError('offline task handoff replay reached a motion entry')
    for name in ('_plan_and_execute_pose', '_command_gripper_position', '_close_gripper',
                 '_execute_grasp6d_plan', '_invoke_plan_bound_action'):
        setattr(node, name, forbid_motion)
    audit = dict(configuration_path=str(CONFIGURATION),
        configuration_sha256=hashlib.sha256(payload).hexdigest(),
        seeded_recorded_history=['original selection and session', 'original committed plan',
            'original executing request', 'original actual arrival settle proof',
            'original plan object geometry'],
        qualified_sources_seeded=False, qualified_acks_seeded=False,
        latest_arrival_object_seeded=False, qualified_phase_seeded=False,
        original_source_association_enabled=True, original_measurement_wait_enabled=True,
        original_phase_creation_enabled=True, motion_entry_sentinels=True,
        inert_constructor_fields=inert_audit,
        inert_constructor_source=str(original_source),
        inert_constructor_source_sha256=hashlib.sha256(original_source.read_bytes()).hexdigest(),
        capture_budget_sec=config.get('unknown_observe_return_requalification_timeout_sec', 3.),
        capture_budget_source='recorded parameter absent; unchanged task default 3 s',
        phase_budget_sec=config['near_field_replan_timeout_sec'])
    return node, config, audit
