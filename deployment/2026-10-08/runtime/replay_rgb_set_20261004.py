"""Replay saved real pixels through unchanged proposal and explicit set tracking."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from alicia_grasp_modes.rgb_observation_region import CenterSearchPolicy, propose_rgb_observation_regions
from alicia_grasp_modes.rgb_observation_set import RgbObservationSetTracker, observation_set_metadata

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
if not Path('/media/zhuyupei/SanStand').is_mount():
    raise RuntimeError('SanStand unavailable')
cv2.setNumThreads(1)
reports=[]
for name in ('rgb_observation_bootstrap_20261004T030534Z', 'rgb_observation_bootstrap_20261004T032657Z'):
    source=ROOT/name/'result.json'
    original=json.loads(source.read_text())
    tracker=RgbObservationSetTracker(); rows=[]
    for row in original['rows']:
        evidence=Path(row['source_file'])
        if hashlib.sha256(evidence.read_bytes()).hexdigest()!=row['source_file_sha256']:
            raise RuntimeError('source artifact hash mismatch')
        with np.load(str(evidence)) as frame:
            bgr=frame['color_bgr'].copy()
        policy=CenterSearchPolicy((.5,.5), radius_fraction=.125)
        started=time.monotonic()
        proposals=propose_rgb_observation_regions(bgr,policy=policy)
        track=tracker.observe(proposals,policy=policy,source_id=row['source_id'],
            generation=row['selection']['generation'],stamp_ns=row['source_stamp_ns'],joints_rad=row['q_source'])
        rows.append(dict(source_stamp_ns=row['source_stamp_ns'],proposal_reason=proposals.reason,
            original_ready=row['observation_interest_ready'], set_ready=track.confirmed,
            reason=track.reason, consistent_frames=track.consistent_frames,
            members=observation_set_metadata(track.regions) if track.regions else [],
            elapsed_sec=time.monotonic()-started))
    report=dict(source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        frame_count=len(rows),original_ready=sum(r['original_ready'] for r in rows),
        set_ready=sum(r['set_ready'] for r in rows),
        member_counts=dict(Counter(len(r['members']) for r in rows)),
        robot_commands_sent=False,measured_target_claimed=False,rows=rows)
    path=ROOT/name/'explicit_rgb_set_replay.json'
    path.write_text(json.dumps(report,indent=2)+'\n')
    reports.append({k:v for k,v in report.items() if k!='rows'})
print(json.dumps(reports,indent=2))
