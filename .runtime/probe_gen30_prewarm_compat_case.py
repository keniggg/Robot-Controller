"""Offline original five-test GEN29/current causal comparison; no ROS calls."""
import importlib.util,json,sys
from pathlib import Path
import pytest
import test_unknown_stream
W=Path(__file__).resolve().parents[1]
which,out=sys.argv[1],Path(sys.argv[2]);out.mkdir()
if which=='baseline':
 p=W/'.runtime/gen30_baseline_source/src/alicia_grasp_modes/scripts/unknown_tabletop_perception.py'
 spec=importlib.util.spec_from_file_location('unknown_stream_test_baseline',p)
 mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
 test_unknown_stream.module=mod
else:mod=test_unknown_stream.module
from alicia_flexible_grasp.vision.rgbd_rectification import PairedPinholeRectification
mp=pytest.MonkeyPatch()
try:
 node,clock,feed,joint,params=test_unknown_stream.rgb_interest_node(mp)
 camera=params['/camera'];mapper=PairedPinholeRectification(dict(camera,model='none',coeffs=[0.]*5))
 camera['runtime_profile_geometry']=dict(geometry_intrinsics=mapper.output_intrinsics,rectification=mapper.contract,geometry_activation_stamp_ns=1)
 statuses=[]
 for _ in range(3):
  feed(target_depth=.35);clock[0]+=.2
  statuses.append(json.loads(node.pubs['detector_status'].publish.call_args[0][0].data))
 diagnostic=dict(node_path=str(Path(mod.__file__).resolve()),anchor_present=node.tracker.anchor is not None,lost=node.tracker.lost,statuses=statuses,camera=camera)
 (out/'diagnostic.json').write_text(json.dumps(diagnostic,indent=2)+'\n')
 print('DIAGNOSTIC',which,diagnostic['anchor_present'],[s.get('detail') for s in statuses],flush=True)
finally:mp.undo()
rc=pytest.main(['-q','src/alicia_grasp_modes/tests/test_measured_face_freeze_lock_independent.py','-p','no:cacheprovider','--tb=short','--basetemp='+str(out/'pytest_tmp'),'--junitxml='+str(out/'pytest.xml')])
sys.exit(int(rc))
