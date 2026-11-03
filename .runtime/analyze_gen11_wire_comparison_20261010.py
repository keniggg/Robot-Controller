"""Read sealed joint-only bag windows; reconstruct, never transmit, SDK frames."""
from pathlib import Path
import binascii
import hashlib
import json
import math
import re

import rosbag
import rospy

ROOT = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
NEW = ROOT/'ros_log/full_latest_restart_20261010T002347Z'
OUT = NEW/'gen11_handoff_analysis'
WORK = Path(__file__).resolve().parents[1]
assert Path('/media/zhuyupei/SanStand').is_mount()
WINDOWS = {
    'gen8': ('20261010T031939.390394Z_5136b6b5d69d49a3bd1ba57514758965', 1791602411., 1791602414.),
    'gen11': ('20261010T045823.108983Z_afb71d5ad1424dad9de6275837bc5f78', 1791608314., 1791608320.),
}
TOPICS = ['/alicia_d/sdk_command', '/alicia_d/accepted_joint_states', '/alicia_d/control_reference']
NAMES = ['Joint%d' % i for i in range(1, 7)]
rows = {}
for label, (record, low, high) in WINDOWS.items():
    directory = ROOT/'grasp_records'/record
    assert json.loads((directory/'manifest.json').read_text())['recorder_closed'] is True
    groups = {topic: [] for topic in TOPICS}
    with rosbag.Bag(str(directory/'keyframes.bag'), 'r') as bag:
        for topic, message, received in bag.read_messages(topics=TOPICS,
                start_time=rospy.Time.from_sec(low), end_time=rospy.Time.from_sec(high)):
            values = dict(zip(message.name, message.position))
            groups[topic].append(dict(stamp_ns=message.header.stamp.to_nsec(),
                received_ns=received.to_nsec(), q=[values[name] for name in NAMES],
                finger_m=values.get('right_finger')))
    rows[label] = dict(record=str(directory), read_range_sec=[low, high], topics=groups)

logs, config = {label: [] for label in WINDOWS}, []
for number, raw in enumerate((NEW/'core.log').open(errors='replace'), 1):
    if re.search('temp|overheat|温度', raw, re.I):
        continue
    clean = re.sub(r'\x1b\[[0-9;]*m', '', raw).rstrip()
    if '/alicia_d_driver_node/joint_speed_deg_s:' in clean:
        config.append(dict(line=number, text=clean))
    if 'Streaming SDK command (ok)' not in clean:
        continue
    match = re.search(r'\[(179160\d+\.\d+)\].*gripper_raw=(\d+) crc=0x([0-9A-Fa-f]+)', clean)
    if not match:
        continue
    stamp = float(match.group(1))
    for label, (_, low, high) in WINDOWS.items():
        if low <= stamp < high:
            logs[label].append(dict(line=number, stamp_sec=stamp,
                gripper_raw=int(match.group(2)), crc=int(match.group(3), 16), text=clean))

assert config and all('15.0' in item['text'] for item in config)
speed = round((15.0/(360./4096.))/50.)*50
assert speed == 150

def reconstructed_frame(q, gripper):
    frame = [0xaa, 0x06, 0x03, 0x1c]
    # q is already decoded successful wire state: round recovers its exact count.
    words = [round((value+math.pi)/(2*math.pi)*4096) for value in q]
    for word in words:
        frame.extend((word & 255, word >> 8, speed & 255, speed >> 8))
    frame.extend((gripper & 255, gripper >> 8, 5500 & 255, 5500 >> 8))
    frame.extend((binascii.crc32(bytes(frame[1:])) & 255, 0xff))
    return frame, words

summary = {}
for label, item in rows.items():
    groups = item['topics']
    accepted, sdk = groups[TOPICS[1]], groups[TOPICS[0]]
    assert accepted and sdk
    def delta(samples):
        q0 = samples[0]['q']
        return [max(abs(row['q'][i]-q0[i]) for row in samples) for i in range(6)]
    matches = []
    for log in logs[label]:
        nearest = min(sdk, key=lambda row: abs(row['stamp_ns']*1e-9-log['stamp_sec']))
        frame, words = reconstructed_frame(nearest['q'], log['gripper_raw'])
        matches.append(dict(log_line=log['line'], source_stamp_ns=nearest['stamp_ns'],
            stamp_difference_sec=log['stamp_sec']-nearest['stamp_ns']*1e-9,
            joint_position_words=words, speed_words=[speed]*6,
            reconstructed_frame_hex=' '.join('%02X'%v for v in frame),
            reconstructed_crc=frame[-2], logged_crc=log['crc'], crc_match=frame[-2]==log['crc']))
    summary[label] = dict(record=item['record'], range_sec=item['read_range_sec'],
        accepted_count=len(accepted), sdk_write_count=len(sdk),
        accepted_max_delta_from_first_rad=delta(accepted),
        sdk_max_delta_from_first_rad=delta(sdk), log_crc_comparisons=matches)

source_paths = [
    'src/real-arm/alicia_d_driver/src/alicia_d_driver_node.cpp',
    'src/real-arm/alicia_d_driver/src/serial_communicator.cpp',
    'src/real-arm/alicia_d_driver/launch/alicia_d_bringup.launch',
    'src/Alicia-D-SDK-use/alicia_d_sdk/hardware/servo_driver.py',
    'src/Alicia-D-SDK-use/alicia_d_sdk/hardware/serial_comm.py',
]
report = dict(summary=summary, launch_speed_evidence=config,
    speed_mapping=dict(requested_deg_s=15., wire_ticks_s=150, bytes_hex='96 00',
        decoded_deg_s=150*360/4096, sdk_range_ticks_s=[50,5000], sdk_step_ticks_s=50),
    positive_enable=dict(driver_hex='AA 05 00 01 01 F9 FF', official_sdk_hex='AA 05 00 01 01 F9 FF', identical=True),
    provenance=dict(source_sha256={rel:hashlib.sha256((WORK/rel).read_bytes()).hexdigest() for rel in source_paths},
        core_log=str(NEW/'core.log'), bag_topics_read=TOPICS),
    conclusions=[
        'Both launch evidence and source use nonzero speed 150 ticks/s; official SDK uses the same mapping and framing at 15 deg/s.',
        'The serial API returning full frame length is the condition for sdk_command publication; this is host write completion, not device ACK.',
        'GEN8 narrow window demonstrates accepted encoder movement with the same source-bound speed/frame construction; GEN11 selected window has none.',
        'CRC reconstruction is corroboration from decoded wire positions and source, not an independent raw TX capture, nor unique speed proof from an 8-bit CRC.',
        'No local SDK evidence establishes a position deadband that explains GEN11. Do not alter amplitude, speed, direction threshold, or timeout from this evidence alone.',
        'User later reported manual control was still held during the failed attempt and is now released. That new现场 fact is separate from recorded GUI ownership flags; this report does not prove causal arbitration at the device.',
    ])
for name, value in [('actuation_wire_gen8_gen11_trace.json', dict(rows=rows, logs=logs)),
                    ('actuation_wire_gen8_gen11_comparison.json', report)]:
    destination = OUT/name
    with destination.open('x') as output:
        json.dump(value, output, ensure_ascii=False, indent=2)
        output.write('\n')
    print(json.dumps(dict(path=str(destination), sha256=hashlib.sha256(destination.read_bytes()).hexdigest())))
print(json.dumps(summary))
