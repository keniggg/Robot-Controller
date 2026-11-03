#!/usr/bin/env python3
"""Offline record extraction only; no ROS node. Geometry uses saved active SDK profile, never camera.yaml."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('offline_extract',HERE/'analyze_approach_failure_20260927.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.RECORD=module.ROOT/'grasp_records/20260927T071553.378661Z_f557b777f3ee4ad48b4e05890fdaac34'
module.OUT=module.ROOT/'ros_log/unknown_grasp_20260923/hover_speed_failure_20260927'
module.T0,module.T1=1790493353.,1790493705.
with contextlib.redirect_stdout(io.StringIO()):module.main()
d=json.loads((module.OUT/'bag_forensics.json').read_text())
s=json.loads((module.OUT/'bag_summary.json').read_text())
for topic in ['/joint_states','/alicia_d/accepted_joint_states','/alicia_d/sdk_command','/alicia_d/control_reference','/joint_commands']:
 rows=[r for r in d['series'][topic] if 1790493688.2547247<=r['t']<=1790493696.6694613]
 s[topic]={'window_sec':[1790493688.2547247,1790493696.6694613],'count':len(rows),'start':rows[0] if rows else None,'end':rows[-1] if rows else None}
(module.OUT/'bag_summary.json').write_text(json.dumps(s,indent=2))
print(module.OUT)
