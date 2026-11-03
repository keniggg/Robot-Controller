#!/usr/bin/env python3
"""Pure AST control-flow probes; no ROS imports, transport, hardware, or bag reads."""
import ast
from contextlib import nullcontext
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
P = W/'src/alicia_grasp_modes'
F = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen41_restart_20261018T011222Z/gen58_resume_r1/observation_pose_fix/held_lattice_fix_r1/seed_wait_fix_r1/contact_feedback_fix_r1/state_contract_audit_r1')

def load(path, names, env):
    tree = ast.parse(path.read_text())
    nodes = []
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names:
            nodes.append(node)
        elif isinstance(node, ast.ClassDef):
            nodes.extend(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name in names)
    assert {n.name for n in nodes} == set(names)
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(path), 'exec'), env)

record = json.loads((F/'r62_first_loss_and_predecessor.json').read_text())
bad = json.loads(record['first_loss']['data']['data'])
good = json.loads(record['last_ready']['data']['data'])
loss = bad['first_loss']
now = loss['observed_stamp_ns']
class Stamp:
    def __init__(self, value): self.value = value
    def to_nsec(self): return self.value
class Pub:
    def __init__(self): self.messages = []
    def publish(self, value): self.messages.append(value)
class Object:
    def __init__(self): self.header = NS(stamp=None, frame_id='')
env = dict(np=np, deepcopy=deepcopy, json=json, STRATEGIES=('two_stage','direct','observe_return'),
    rospy=NS(Time=NS(now=lambda:Stamp(now)), logwarn=lambda *a:None),
    ObjectPose=Object, Bool=lambda **kw:NS(**kw))
load(P/'src/alicia_grasp_modes/loss_observation.py', ['first_loss_observation'], env)
load(P/'src/alicia_grasp_modes/tabletop.py', ['TargetTracker','_matching_candidates'], env)
load(P/'src/alicia_grasp_modes/selection.py', ['parse_selection','validate_mode_strategy'], env)
load(P/'src/alicia_grasp_modes/runner_contract.py', ['unknown_target_loss_reason'], env)
load(P/'scripts/unknown_tabletop_perception.py', ['unavailable'], env)
tracker = env['TargetTracker'](NS(association_distance_m=.1))
anchor = NS(position_base=np.asarray(loss['anchor_position_base']), extent_m=np.asarray(loss['anchor_extent_m']))
tracker.anchor = tracker.last = anchor
tracker.plane_base = np.asarray(loss['reference_support_plane_base'])
source = [NS(header=NS(stamp=Stamp(loss['source_stamp_ns']))) for _ in range(2)]
node = NS(input_lock=nullcontext(), mode='unknown', tracker=tracker, generation=loss['generation'],
    selection_ns=loss['selection_stamp_ns'], first_loss=None,
    anchor_source_stamp_ns=loss['anchor_source_stamp_ns'],
    last_good_source_stamp_ns=loss['last_good_source_stamp_ns'],
    last_consumed_ns=loss['last_consumed_source_stamp_ns'], last_shape=(2,3),
    base_frame='base_link', camera_frame='camera_link', _clear_rgb_interest=lambda *a:None,
    bridge=NS(cv2_to_imgmsg=lambda array, **kw:NS(header=NS(), pixels=array)),
    pubs={k:Pub() for k in ('object_mask','object','object_detected','raw_object_detected')},
    status=lambda *a:None, _station_compute_registration=deepcopy(good['station_compute_wait']))
env['unavailable'](node, bad['detail'], source[0].header.stamp, source)
assert node.first_loss == loss
assert tracker.lost and node._station_compute_registration is None
same, reason = tracker.choose(NS(candidates=[anchor]), (2,3))
assert same is None and reason == 'target_lost_requires_new_generation'
assert node.pubs['object'].messages[-1].detected is False
assert not np.any(node.pubs['object_mask'].messages[-1].pixels)

load(P/'src/alicia_grasp_modes/station_compute_wait_task.py', ['_station_compute_task_status'], env)
retired = []
selected = dict(mode='unknown', strategy='observe_return', generation=6, stamp_ns=100)
consumer = NS(_station_task_availability_route=lambda *a:False,
    _station_handoff_pending=lambda *a:False, _station_task_handoff=None,
    _station_task_receiving=lambda:True, _station_task_record={'contract':{'source_ns':200}},
    _grasp6d_plan_guard=nullcontext, _station_task_check=lambda:None,
    _mode_bound_selection=selected, _mode_selection=selected,
    _station_task_retire=lambda reason:retired.append(reason))
env['STATION_REQUALIFICATION']='station_requalification'
old = dict(mode='unknown',generation=6,selection_stamp_ns=100,
           source_stamp_ns=150,state='unavailable',target_locked=True,target_lost=True)
env['_station_compute_task_status'](consumer,old)
assert retired == ['producer reported target lost']
assert env['unknown_target_loss_reason'](old,selected,now) == ''

contract = good['station_compute_wait']
result = dict(passed=True, scope='finite control-flow reproduction, not numerical geometry replay',
    production_edited=False, ros_imports_or_transport=False, bag_read=False,
    actual_rejection_to_terminal=dict(first_loss_exact_match=True,
        lost_tracker_rejects_restored_same_candidate=True, failed_frame_false_empty=True,
        original_identity_remaining_sec=(contract['identity_deadline_ns']-now)*1e-9,
        original_geometry_expired_sec=(now-contract['geometry_deadline_ns'])*1e-9,
        rejected_source_age_sec=(now-loss['source_stamp_ns'])*1e-9),
    distinct_static_gap=dict(source_older_than_current_registration=True,
        malformed_terminal_retires_station_callback=True,
        same_packet_rejected_by_canonical_loss_validator=True,
        actual_r62_cause=False),
    caveats=['Context/safety gates in the consumer routing probe are inert valid-context stubs.',
             'A valid irreversible same-selection tombstone must not be ignored solely for age.',
             'The overlap geometry predicate is retained; this probe does not establish why it fired.'])
(F/'contract_probes.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False))
