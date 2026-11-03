#!/usr/bin/env python3
"""Seal source-only diagnostic limits and original replay deadlines; no ROS."""
import hashlib
import json
from pathlib import Path

N = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_contact_admission_20261013T011103Z')
F = N / 'verification/gen14_arrival_independent'
D = N / 'verification/gen14_arrival_diagnosis/source_local_face_measured_plane_r3/result.json'
W = Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
PRE = 1791860176945733070

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    fixture_path = F / 'fixture.json'
    full_path = F / 'full_arrival_frames/fixture.json'
    status_path = F / 'all_detector_statuses.json'
    fixture = json.loads(fixture_path.read_text())
    full = json.loads(full_path.read_text())
    statuses = json.loads(status_path.read_text())
    diagnostic = json.loads(D.read_text())
    status = next(row['message'] for row in statuses if row['message'].get('source_stamp_ns') == PRE and row['message'].get('state') == 'ready')
    timing = status['frame_timing']
    component = timing['rgb_components']['hold.total'][0]
    entry = timing['processing_start_ros_ns'] + round(component['start_elapsed_sec'] * 1e9)
    rows = []
    for row in diagnostic['rows']:
        results = {x['method']: x['evidence']['code'] for x in row['results']}
        rows.append(dict(source_ns=row['source_ns'], original_capture_remaining_from_source_sec=(full['capture_deadline_ns']-row['source_ns'])*1e-9, diagnostic_results=results))
    report = dict(
        scope='Read-only boundary audit. Source-local face diagnostic is not whole-object qualification; no candidate has been executed by this audit.',
        premotion=dict(source_ns=PRE, recorded_hold_entry_ns=entry,
            recorded_entry_source_age_sec=(entry-PRE)*1e-9,
            original_source_deadline_ns=PRE+800000000,
            remaining_source_budget_at_entry_sec=(PRE+800000000-entry)*1e-9,
            original_hold_wall_sec=component['wall_sec'],
            require_new_freezing_and_owner_recheck_cost_charged=True),
        arrival=dict(arrival_ns=full['arrival_ns'], capture_deadline_ns=full['capture_deadline_ns'],
            capture_budget_sec=(full['capture_deadline_ns']-full['arrival_ns'])*1e-9,
            source_max_age_sec=.8, saved_exact_pairs=len(full['frames']),
            first_saved_source_ns=full['frames'][0]['source_ns'],
            last_saved_source_ns=full['frames'][-1]['source_ns'],
            callback_wire=[dict(state=row['message']['state'], received_ns=row['received_ns'], stamp_ns=row['message']['stamp_ns']) for row in fixture['wire'] if row['topic'].endswith('measurement_request')]),
        face_diagnostic=dict(fixed_affine_pass_count=sum(x['diagnostic_results']['affine']=='DENSE_TEXTURE_CORROBORATED' for x in rows),
            count=len(rows), rows=rows, reference_face=diagnostic['reference_face'],
            original_frame_time_not_measured_by_this_diagnostic=True,
            extra_lifetime_not_authorized=True,
            whole_object_geometry_or_three_frame_acceptance_not_proven=True),
        replay_requirements=['Original accepted PRE is explicitly seeded only as the historical starting condition; no arrival acceptance is seeded.',
            'All saved pairs are supplied with original receive times; latest-only scheduling may consume a subset and must list it truthfully.',
            'New PRE preparation, callbacks, geometry, dense checks, SDK/TF waits and final commit time advance a monotonic modeled clock.',
            'Three distinct actually accepted sources and real qualified callback must occur before original capture deadline; rejections reset the original confirmation as production dictates.',
            'Conditional task phase is 60 seconds and explicitly does not replay the remote participant or physical execution.',
            'Short capture replay alone does not exercise 5-second station or 60-second phase terminal boundaries; separate boundary tests remain necessary.'],
        dependencies_sha256={str(path): sha(path) for path in [fixture_path,full_path,status_path,D,Path(__file__),W/'.runtime/replay_gen14_actual_qualification_v3_20261013.py']},
        robot_actions=False, ros_connections=False)
    assert report['arrival']['capture_budget_sec']==3.0
    assert report['arrival']['saved_exact_pairs']==57
    assert report['face_diagnostic']['fixed_affine_pass_count']==9
    output=F/'replay_deadline_requirements.json'
    with output.open('x') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2)
        stream.write('\n')
    print(str(output),sha(output))

if __name__=='__main__':
    main()
