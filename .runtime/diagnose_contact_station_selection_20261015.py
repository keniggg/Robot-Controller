"""Offline replay of the recorded request; no ROS graph or hardware access."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import argparse
import hashlib
import json
import threading

import rospy
from std_msgs.msg import String
from alicia_grasp_modes.contact_station_retirement import parse_request
from alicia_grasp_modes.contact_station_producer import ContactStationProducerMixin
from alicia_grasp_modes.selection import parse_selection

p = argparse.ArgumentParser()
p.add_argument('--output', required=True)
args = p.parse_args()
root = Path(__file__).resolve().parents[1]
session = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/full_latest_gen22_restart_20261015T003700Z')
parameter_path = session/'verification/contact_station_param_types/result.json'
events_path = session/'verification/contact_station_attempt1_evidence/handoff_and_task.json'
selection = json.loads(parameter_path.read_text())['selection']
events = json.loads(events_path.read_text())
event = next(row for row in events if row['topic'] == '/grasp_mode/contact_station_retirement_request')
request = event['data']
if isinstance(request, str):
    request = json.loads(request)
if set(request) == {'data'}:
    request = json.loads(request['data'])
now = request['stamp_ns'] + 1
result = dict(kind='offline_actual_contact_selection_replay', request=request,
    raw_selection=selection, source_ns=now, ros_graph_access=False,
    evidence_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                     for path in (parameter_path, events_path)}, callers={})
for caller, selected in [('producer_raw_xmlrpc', selection),
                         ('remote_canonical', parse_selection(selection)),
                         ('task_canonical', parse_selection(selection))]:
    try:
        parsed = parse_request(request, selected, now)
        row = dict(accepted=True, request_unchanged=parsed == request)
    except Exception as exc:
        row = dict(accepted=False, error_type=type(exc).__name__, reason=str(exc))
    result['callers'][caller] = row

messages = []
node = SimpleNamespace(lock=threading.RLock(), input_lock=threading.RLock(),
    _rgb_interest_epoch=request['control_epoch_ns'],
    _station_compute_task_state=SimpleNamespace(state='MOVE_PREGRASP', active=True),
    _contact_station_ack_pub=SimpleNamespace(publish=lambda msg: messages.append(json.loads(msg.data))),
    _contact_station_context=lambda req: ({}, {}))
def parameter(key, default=None):
    if key == '/grasp_mode/selection':
        return deepcopy(selection)
    if key == '/gui/joint_direct_mode':
        return False
    raise AssertionError('unexpected parameter query: ' + key)
with patch.object(rospy, 'get_param_cached', parameter), patch.object(rospy.Time, 'now',
        return_value=rospy.Time(now//10**9, now%10**9)):
    ContactStationProducerMixin._contact_station_request_cb(node, String(data=json.dumps(request)))
result['producer_callback'] = dict(acks=messages, prepared=getattr(node, '_contact_station_prepared', None),
    scope='real callback and parser; owner context deliberately stubbed after parsing only')
source = root/'src/alicia_grasp_modes/src/alicia_grasp_modes/contact_station_retirement.py'
result['source_sha256'] = {str(source): hashlib.sha256(source.read_bytes()).hexdigest()}
out = Path(args.output)
out.mkdir(parents=True, exist_ok=False)
(out/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)+'\n')
print(json.dumps(dict(callers=result['callers'], producer_ack_count=len(messages),
                     output=str(out/'result.json')), ensure_ascii=False, sort_keys=True))
