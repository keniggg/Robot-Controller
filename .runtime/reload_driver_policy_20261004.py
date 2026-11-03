"""Apply the authorized temperature-off policy without disabling any joint.

Only the driver process is replaced. Preserve the actual before/after epoch;
this deployment report is not a target confirmation or motion authorization.
"""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import threading
import time
import xmlrpc.client

import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
from alicia_flexible_grasp_supervisor.msg import GraspState
from alicia_grasp_modes.selection import parse_selection
from alicia_grasp_modes.rgb_observation_source import digest
import sys

WORK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORK/'tools/diagnostics'))
from prepare_rgb_acquisition_retry import retry_source_context

EXT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RUN = EXT/'ros_log/full_ros_resume_20261004T002930Z'
DRIVER = '/alicia_d_driver_node'
PRESERVED = ('/bessica_d_hw_interface', '/supervisor_camera',
    '/unknown_tabletop_perception', '/grasp_task_node',
    '/remote_grasp6d_node', '/motion_gateway')
EXPECTED = '319c8d24b55a930dddf2f7c72145fa4652eacebef54a4c8d08591a12bca44e9f'
BINARY = WORK/'devel/lib/alicia_d_driver/alicia_d_driver_node'


def node(name):
    uri = rosgraph.Master('/driver_policy_reload').lookupNode(name)
    code, message, pid = xmlrpc.client.ServerProxy(uri).getPid('/driver_policy_reload')
    if code != 1:
        raise RuntimeError(message)
    return dict(uri=uri, pid=pid)


def snapshot():
    nodes = {name: node(name) for name in (DRIVER,)+PRESERVED}
    epoch = rospy.wait_for_message('/alicia_d/control_reference_epoch', Header, timeout=4)
    selection = parse_selection(rospy.get_param('/grasp_mode/selection'))
    ledger = rospy.get_param('/rgb_acquisition/ledger')
    context = retry_source_context(selection, epoch.stamp.to_nsec(),
        rospy.get_param('/camera'), rospy.get_param('/handeye', {}),
        rospy.get_param('/robot_description'))
    binary_sha = hashlib.sha256(Path('/proc', str(nodes[DRIVER]['pid']), 'exe').read_bytes()).hexdigest()
    sample = rospy.wait_for_message('/alicia_d/accepted_joint_states', JointState, timeout=4)
    values = dict(zip(sample.name, sample.position))
    if sample.header.frame_id != 'sdk_measured':
        raise RuntimeError('Actual SDK sample required')
    return dict(nodes=nodes, epoch_ns=epoch.stamp.to_nsec(), selection=selection,
        ledger=ledger, source_context=context, driver_binary_sha256=binary_sha,
        feedback=dict(stamp_ns=sample.header.stamp.to_nsec(), frame_id='sdk_measured',
            positions=[values['Joint'+str(index)] for index in range(1, 7)]))


def main():
    if not EXT.parent.parent.is_mount():
        raise RuntimeError('SanStand unavailable; no fallback')
    if hashlib.sha256(BINARY.read_bytes()).hexdigest() != EXPECTED:
        raise RuntimeError('Tested driver binary changed')
    rospy.init_node('authorized_driver_policy_reload', anonymous=True, disable_signals=True)
    if rospy.wait_for_message('/grasp/state', GraspState, timeout=4).active:
        raise RuntimeError('Active grasp owns driver')
    if rospy.get_param('/rgb_acquisition/ledger', {}).get('phase') in ('reserved', 'executing'):
        raise RuntimeError('Active observation owns driver')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    out = RUN/('driver_temperature_off_'+stamp)
    out.mkdir()
    log = out/'driver.log'
    evidence = out/'feedback_and_transport_codes.jsonl'
    report_path = out/'result.json'
    reference = EXT/'ros_log/rgb_observation_new_trial_20261004T063140.568948Z/result.json'
    original = json.loads(reference.read_text())
    transitions = [json.loads((EXT/('ros_log/rgb_observation_new_trial_'+name+'/result.json')).read_text())
        for name in ('20261004T072512.280310Z', '20261004T075538.835989Z')]
    rgb_record = json.loads((EXT/'ros_log/rgb_observation_bootstrap_20261004T061706Z/result.json').read_text())
    source_ns = 1791094630503762245
    row = next(row for row in rgb_record['rows'] if row['source_stamp_ns'] == source_ns)
    report = dict(version=1, kind='authorized_driver_only_reload', status='started',
        authorization_reference='User: 以后不要管机械臂温度的情况了，不要查询也不要对其反应',
        driver_node=DRIVER, original_transition_sha256=digest(original),
        confirmation_receipt_sha256=digest(original['alignment_receipt']),
        unreserved_transition_sha256=[digest(record) for record in transitions],
        aligned_source_stamp_ns=source_ns, aligned_rgb_sha256=row['source_rgb_sha256'],
        policy_after=dict(software_temperature_monitoring_enabled=False, temperature_poll_rate_hz=0.0),
        actuation=dict(joint_motion_commands_sent=False, torque_disable_commands_sent=False,
            torque_enable_policy='positive_enable_only', positive_enable_observed=False,
            evidence_files=[]),
        old_geometry_reused=False, old_epoch_preserved=False,
        historical_continuous_stationarity_claimed=False)
    records = []
    lock = threading.Lock()
    def save(kind, value):
        with lock:
            records.append(dict(kind=kind, received_ns=rospy.Time.now().to_nsec(), **value))
    def joint(message):
        save('accepted_feedback', dict(stamp_ns=message.header.stamp.to_nsec(),
            frame_id=message.header.frame_id, names=list(message.name), positions=list(message.position)))
    def diagnostic(message):
        # Transport codes only: do not inspect or retain temperature values.
        for status in message.status:
            if status.name == 'alicia_d/sdk_raw_frame':
                code = next((v.value for v in status.values if v.key == 'command_function_hex'), '')
                save('transport_code', dict(stamp_ns=message.header.stamp.to_nsec(), code=code))
    subscribers = [rospy.Subscriber('/alicia_d/accepted_joint_states', JointState, joint, queue_size=100),
        rospy.Subscriber('/alicia_d/sdk_diagnostics', DiagnosticArray, diagnostic, queue_size=100),
        rospy.Subscriber('/alicia_d/control_reference_epoch', Header,
            lambda msg: save('epoch', dict(stamp_ns=msg.stamp.to_nsec())), queue_size=10),
        rospy.Subscriber('/alicia_d/actuation_status', String,
            lambda msg: save('actuation_status', dict(data=msg.data)), queue_size=10)]
    try:
        report['before'] = snapshot()
        report['started_ns'] = rospy.Time.now().to_nsec()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        rospy.set_param(DRIVER+'/software_temperature_monitoring_enabled', False)
        rospy.set_param(DRIVER+'/temperature_poll_rate_hz', 0.0)
        previous = report['before']['nodes'][DRIVER]
        try:
            xmlrpc.client.ServerProxy(previous['uri']).shutdown('/driver_policy_reload',
                'Apply user-requested temperature-off software policy; no torque-disable')
        except OSError as exc:
            report['shutdown_reply_error'] = str(exc)
        deadline = time.monotonic()+12
        while Path('/proc', str(previous['pid'])).exists() and time.monotonic() < deadline:
            time.sleep(.1)
        if Path('/proc', str(previous['pid'])).exists():
            raise RuntimeError('Old driver did not exit; no duplicate serial owner started')
        script = WORK/'.runtime/start_temperature_off_driver_20261004.sh'
        subprocess.run(['systemd-run', '--user', '--collect',
            '--unit=alicia-temperature-off-driver-'+stamp.lower(),
            '--property=PartOf=alicia-ros-live-20260928.service', '--property=Restart=no',
            '--property=WorkingDirectory='+str(WORK),
            '--property=StandardOutput=append:'+str(log),
            '--property=StandardError=append:'+str(log), '/bin/bash', str(script)],
            check=True, timeout=15)
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            try:
                if node(DRIVER)['pid'] != previous['pid']:
                    break
            except Exception:
                pass
            time.sleep(.2)
        else:
            raise RuntimeError('New driver not registered')
        time.sleep(2)
        report['after'] = snapshot()
        report['completed_ns'] = rospy.Time.now().to_nsec()
        before, after = report['before'], report['after']
        if any(before['nodes'][n] != after['nodes'][n] for n in PRESERVED):
            raise RuntimeError('Unrelated process changed')
        if before['selection'] != after['selection'] or before['ledger'] != after['ledger']:
            raise RuntimeError('Selection or observation owner changed during maintenance')
        if after['driver_binary_sha256'] != EXPECTED:
            raise RuntimeError('New process binary does not match tested artifact')
        report['status'] = 'completed'
        report['after_motion_enabled'] = rospy.wait_for_message('/alicia_d/motion_enabled', Bool, timeout=4).data
        report['after_feedback_ready'] = rospy.wait_for_message('/alicia_d/feedback_ready', Bool, timeout=4).data
        print(json.dumps(dict(driver_reloaded=True, report=str(report_path),
            before_epoch=before['epoch_ns'], after_epoch=after['epoch_ns'],
            motion_enabled=report['after_motion_enabled'], feedback_ready=report['after_feedback_ready'])), flush=True)
        time.sleep(30)
    except Exception as exc:
        report.update(status='failed', failure=str(exc))
        raise
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        with lock:
            saved = list(records)
        evidence.write_text(''.join(json.dumps(row)+'\n' for row in saved))
        report['actuation']['evidence_files'] = [dict(path=str(evidence), sha256=hashlib.sha256(evidence.read_bytes()).hexdigest())]
        if log.exists():
            sealed_log = out/'driver_startup_snapshot.log'
            sealed_log.write_bytes(log.read_bytes())
            report['actuation']['positive_enable_observed'] = 'torque-on will be requested' in sealed_log.read_text(errors='replace')
            report['actuation']['positive_enable_evidence_scope'] = 'driver startup request log; not hardware ACK or measured actuation confirmation'
            report['actuation']['evidence_files'].append(dict(path=str(sealed_log), sha256=hashlib.sha256(sealed_log.read_bytes()).hexdigest()))
        if 'after' in report:
            cutoff = report['after']['feedback']['stamp_ns']
            rows = [r for r in saved if r.get('stamp_ns', 0) >= cutoff]
            feedback = [r['stamp_ns'] for r in rows if r['kind'] == 'accepted_feedback']
            gaps = [(b-a)/1e6 for a,b in zip(feedback, feedback[1:])]
            report['verification'] = dict(accepted_feedback_count=len(feedback),
                max_accepted_feedback_gap_ms=max(gaps) if gaps else None,
                transport_codes={code: sum(r.get('code') == code for r in rows)
                    for code in set(r.get('code') for r in rows if r['kind'] == 'transport_code')})
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(dict(report=str(report_path), status=report['status'],
            verification=report.get('verification')), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
