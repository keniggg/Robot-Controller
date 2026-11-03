"""Read-only sealed-record analysis and offline replay of the exact driver guard."""
from pathlib import Path
import hashlib
import json
import math
import subprocess
import tempfile

import numpy as np
import rosbag

WORK = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = EXT / 'ros_log/full_latest_restart_20261010T002347Z'
REC = EXT / 'grasp_records/20261010T045823.108983Z_afb71d5ad1424dad9de6275837bc5f78'
assert Path('/media/zhuyupei/SanStand').is_mount()
manifest = json.loads((REC / 'manifest.json').read_text())
assert manifest['state'] == 'complete' and manifest['recorder_closed']
accepted, commands, controller, status, goals, results = [], [], [], [], [], []
with rosbag.Bag(str(REC / 'keyframes.bag')) as bag:
    for topic, message, stamp in bag.read_messages(topics=[
            '/alicia_d/accepted_joint_states', '/alicia_d/sdk_command',
            '/alicia_controller/state', '/alicia_d/actuation_status',
            '/alicia_controller/follow_joint_trajectory/goal',
            '/alicia_controller/follow_joint_trajectory/result']):
        row = {'received_ns': stamp.to_nsec()}
        if topic in ('/alicia_d/accepted_joint_states', '/alicia_d/sdk_command'):
            row.update(source_ns=message.header.stamp.to_nsec(), q=list(message.position[:6]))
            (accepted if topic.endswith('accepted_joint_states') else commands).append(row)
        elif topic.endswith('/state'):
            row.update(source_ns=message.header.stamp.to_nsec(),
                actual=list(message.actual.positions), desired=list(message.desired.positions),
                error=list(message.error.positions), actual_velocity=list(message.actual.velocities))
            controller.append(row)
        elif topic.endswith('/goal'):
            row.update(goal_id=message.goal_id.id, duration_sec=message.goal.trajectory.points[-1].time_from_start.to_sec(),
                points=[{'q': list(point.positions), 'time': point.time_from_start.to_sec()}
                        for point in message.goal.trajectory.points],
                goal_tolerance=[{'name': t.name, 'position': t.position, 'velocity': t.velocity}
                                for t in message.goal.goal_tolerance],
                path_tolerance=[{'name': t.name, 'position': t.position} for t in message.goal.path_tolerance],
                goal_time_tolerance_sec=message.goal.goal_time_tolerance.to_sec())
            goals.append(row)
        elif topic.endswith('/result'):
            row.update(action_status=message.status.status, error_code=message.result.error_code)
            results.append(row)
        else:
            row['status'] = message.data
            if not status or status[-1]['status'] != message.data:
                status.append(row)

assert len(goals) == len(results) == 1
origin = np.array(accepted[0]['q'])
quantum = 2 * math.pi / 4096
first_changed = next(row for row in commands if np.max(np.abs(np.array(row['q']) - origin)) > 1e-9)
stream_origin = np.array(first_changed['q'])
probe_start = next(row for row in commands if row['source_ns'] >= first_changed['source_ns']
    and np.max(np.abs(np.array(row['q']) - stream_origin)) >= .02)
deadline_ns = probe_start['source_ns'] + 1_000_000_000
window = [row for row in accepted if probe_start['source_ns'] < row['source_ns'] <= deadline_ns]
assert window
window_gaps = [b['source_ns'] - a['source_ns'] for a, b in zip(window, window[1:])]
last_controller = max((row for row in controller if row['received_ns'] <= results[0]['received_ns']),
    key=lambda row: row['received_ns'])
epoch_ns = 1791608319688858540

# No ROS calls: compile the unchanged production state machine and replay only
# accepted-feedback / successful-write source stamps from the sealed bag.
events = [(row['source_ns'], kind, row['q']) for kind, rows in [('f', accepted), ('w', commands)] for row in rows]
events.sort()
cpp_rows = ',\n'.join('{%.9f,\'%s\',{%s}}' % (ns * 1e-9, kind, ','.join('%.17g' % v for v in q))
    for ns, kind, q in events)
header = WORK / 'src/real-arm/alicia_d_driver/include/alicia_d_driver/actuation_confirmation.hpp'
program = r'''
#include <iomanip>
#include <iostream>
#include "HEADER"
struct Event { double stamp; char kind; std::vector<double> q; };
int main() {
  std::vector<Event> events = {ROWS};
  for (int injected_counts : {0, 1, 2, -2}) {
    ActuationConfirmation guard;
    guard.reset_for_positive_enable(events.front().stamp - 1.0);
    bool admitted=false, injected=false, ever_confirmed=false;
    double opened=0.0, confirmed=0.0;
    for (const auto &e:events) {
      guard.update(e.stamp);
      auto q=e.q;
      if(e.kind=='f') {
        if (!injected && opened>0.0 && e.stamp>opened && injected_counts!=0) {
          q[2] += injected_counts * 2.0 * M_PI / 4096.0;
          injected=true;
        }
        guard.note_feedback(q,e.stamp);
      } else {
        if(!admitted) { admitted=guard.admit_command(q,e.stamp,nullptr); }
        guard.note_streamed_target(q,e.stamp);
        if (opened==0.0 && guard.status_text()=="PENDING:AWAITING_ENCODER_RESPONSE") opened=e.stamp;
      }
      if (!ever_confirmed && guard.state()==ActuationState::CONFIRMED) {
        ever_confirmed=true; confirmed=e.stamp;
      }
    }
    std::cout << std::setprecision(17) << injected_counts << " " << opened << " "
      << ever_confirmed << " " << confirmed << " " << guard.status_text() << "\n";
  }
}
'''.replace('HEADER', str(header)).replace('ROWS', cpp_rows)
with tempfile.TemporaryDirectory(prefix='gen11-response-replay-build-') as build:
    executable = str(Path(build) / 'replay')
    subprocess.run(['g++', '-std=c++14', '-O2', '-x', 'c++', '-', '-o', executable],
        input=program, text=True, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    replay_output = subprocess.check_output([executable], text=True)
replays = []
for line in replay_output.strip().splitlines():
    count, opened, ever_confirmed, confirmed, final_status = line.split()
    replays.append(dict(injected_counterfactual_joint3_counts=int(count),
        probe_opened_sec=float(opened), ever_confirmed=bool(int(ever_confirmed)),
        first_confirmed_sec=float(confirmed), final_status=final_status))
assert [r['ever_confirmed'] for r in replays] == [False, False, True, False]
assert abs(replays[0]['probe_opened_sec'] - probe_start['source_ns'] * 1e-9) < 1e-6

files = [header,
    WORK / 'src/real-arm/alicia_d_driver/include/alicia_d_driver/sdk_position_codec.hpp',
    WORK / 'src/real-arm/alicia_d_driver/config/controllers.yaml',
    WORK / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/actuation_bootstrap.py',
    WORK / 'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/robot/moveit_planner.py',
    WORK / 'src/alicia_flexible_grasp_supervisor/scripts/motion_gateway_node.py']
report = dict(record=str(REC), physical_actions=False, production_edited=False,
    manifest_sha256=hashlib.sha256((REC / 'manifest.json').read_bytes()).hexdigest(),
    source_sha256={str(p.relative_to(WORK)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    goal=goals[0], result=results[0], sdk_quantum_rad=quantum,
    command_probe_threshold_rad=.02, measured_response_threshold_rad=.003,
    response_deadline_sec=1.0, first_changed_wire=first_changed, probe_first_eligible_wire=probe_start,
    first_eligible_wire_delta_from_feedback_counts=np.rint((np.array(probe_start['q'])-origin)/quantum).astype(int).tolist(),
    first_eligible_wire_delta_from_stream_origin_counts=np.rint((np.array(probe_start['q'])-stream_origin)/quantum).astype(int).tolist(),
    final_wire_delta_from_feedback_counts=np.rint((np.array(commands[-1]['q'])-origin)/quantum).astype(int).tolist(),
    deadline_source_ns=deadline_ns, retirement_epoch_source_ns=epoch_ns,
    retirement_after_probe_sec=(epoch_ns-probe_start['source_ns'])*1e-9,
    accepted_samples_inside_response_window=len(window), response_window_samples=window,
    first_feedback_after_probe_sec=(window[0]['source_ns']-probe_start['source_ns'])*1e-9,
    last_feedback_after_probe_sec=(window[-1]['source_ns']-probe_start['source_ns'])*1e-9,
    largest_feedback_gap_inside_response_window_sec=max(window_gaps)*1e-9,
    maximum_measured_delta_rad=float(np.max(np.abs(np.array([r['q'] for r in accepted])-origin))),
    last_controller_state_before_action_success=last_controller,
    all_last_controller_endpoint_errors_within_explicit_goal_tolerance=all(
        abs(error)<t['position'] for error,t in zip(last_controller['error'],goals[0]['goal_tolerance'])),
    driver_guard_offline_replays=replays,
    conclusions=[
        'No demonstrated software defect caused premature response timeout in GEN11. The exact unchanged driver guard replay opens the response probe at the same recorded quantized write that precedes AWAITING_ENCODER_RESPONSE.',
        'The three-second prefix is not a one-second motion deadline: the one-second response deadline begins only when cumulative successfully written displacement crosses 0.02 rad. Recorded fresh accepted encoder packets span this entire response opportunity without any position change.',
        'Quantization did not erase the request: Joint3 reached 16 counts (0.0245436926 rad). Because the first one-count change becomes the stream command baseline, the probe opens at 15 counts from initial feedback, i.e. 14 cumulative counts from that baseline, above 0.02 rad. Only a subsequent same-direction two-count encoder displacement (0.0030679616 rad) is sufficient; unchanged feedback is not.',
        'Counterfactual offline guard replays confirm that an otherwise identical recorded sequence with one subsequent two-count Joint3 directional response would have confirmed; one count and a two-count reverse response do not. These are synthetic control cases, not claims of actual motion.',
        'The recorded action goal explicitly sets 0.035-rad position goal tolerances and 0.03-rad/s velocity tolerances, so its 0.025-rad bounded prefix may legitimately finish SUCCEEDED with zero measured displacement. The recorded final controller error and zero actual velocity are within those tolerances. Action success is trajectory tolerance satisfaction only.',
        'The gateway separately requires fresh driver CONFIRMED status after the action; it returned ACTUATION_PREFIX_NO_RESPONSE here. There is no observed false actuation success or full observation continuation. Tightening or loosening action tolerance alone does not make a nonresponding device respond.',
        'The missing bootstrap measurement lease remains a separate potential issue for a future prefix that actually moves, but the GEN11 primary timeout does not justify changing lease, amplitude, response thresholds or deadlines as its claimed root-cause fix.',
    ],
    limitations=[
        'Host successful-write telemetry proves bytes were submitted by the serial writer; it does not prove device acceptance, motor response or trustworthy unstuck encoder telemetry.',
        'This record cannot determine whether the device ignored commands, motor torque was unavailable, or feedback stayed frozen. No temperature reading, live command or automatic recovery is used.',
        'The replay uses accepted packet and successful write source stamps; unrecorded internal timer calls are not reconstructed. The recorded retirement time independently verifies the original one-second window.',
    ])
output = NEW / 'gen11_handoff_analysis/prefix_response_window_review.json'
assert not output.exists()
output.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)+'\n')
print(json.dumps({'path': str(output), 'sha256': hashlib.sha256(output.read_bytes()).hexdigest(),
    'response_window_samples': len(window), 'response_duration_sec': report['retirement_after_probe_sec'],
    'first_feedback_after_probe_sec': report['first_feedback_after_probe_sec'],
    'last_feedback_after_probe_sec': report['last_feedback_after_probe_sec'],
    'last_controller_error_rad': last_controller['error'], 'replays': replays}, ensure_ascii=False))
