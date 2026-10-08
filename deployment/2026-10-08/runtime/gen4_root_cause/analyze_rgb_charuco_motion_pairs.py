"""Offline companion: every usable RGB board pair,including immediate baseline.

Consumes the source-hashed RGB-only analysis; no ROS imports or device access.
Pure RGB rotation does not depend on board scale. Depth remains a separate
measurement,and spatial-fold ranges are conditional sensitivity,not intervals.
"""
import argparse
import hashlib
import importlib.util
import itertools
import json
from pathlib import Path

import cv2
import numpy as np

CORE_PATH=Path(__file__).with_name('analyze_rgb_charuco_coordinate_chain.py')
SPEC=importlib.util.spec_from_file_location('independent_rgb_chain',CORE_PATH)
CORE=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(CORE)


def motion_metrics(first,second):
 result=CORE.compare_frames(first,second)
 matrices={'RGB':np.asarray(result['T_camera0_camera1_RGB_only']),
           'recorded_TF':np.asarray(result['T_camera0_camera1_recorded_TF'])}
 if 'depth_supported_motion'in result:
  matrices['depth_supported']=np.asarray(result['depth_supported_motion']['T_camera0_camera1'])
 vectors={name:cv2.Rodrigues(T[:3,:3])[0].ravel()for name,T in matrices.items()}
 reference=vectors['recorded_TF'];length=np.linalg.norm(reference)
 metrics={}
 for name,v in vectors.items():
  item=dict(rotation_angle_deg=float(np.degrees(np.linalg.norm(v))),
       rotation_vector_deg=np.degrees(v).tolist())
  if length>1e-8:
   axis=reference/length;projection=float(v@axis)
   item.update(rotation_projected_on_TF_axis_deg=float(np.degrees(projection)),
       nonparallel_rotation_deg=float(np.degrees(np.linalg.norm(v-axis*projection))))
  else:item['TF_axis_projection_unavailable']='stationary TF rotation has no defined axis'
  metrics[name]=item
 result['rotation_measurements']=metrics
 if 'depth_supported_motion'in result:
  A={i:p for i,p in zip(first['rgb']['ids'],first['depth']['depth_supported_corners_optical_m'])}
  B={i:p for i,p in zip(second['rgb']['ids'],second['depth']['depth_supported_corners_optical_m'])}
  ids=sorted(set(A)&set(B));C0,C1=np.asarray(first['T_base_optical']),np.asarray(second['T_base_optical'])
  delta=CORE.mappts(C1,[B[i]for i in ids])-CORE.mappts(C0,[A[i]for i in ids])
  result['depth_fixed_board_base_point_drift']=dict(common_ids=ids,
      point_drift_mm=CORE.error_stats(np.linalg.norm(delta,axis=1),1000),
      centroid_shift_xyz_mm=(delta.mean(0)*1000).tolist(),
      interpretation='Measured-depth plane intersections at common RGB corner IDs; '
       'independent from RGB PnP but shares source pixels and camera rays.')
 return result


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--analysis',type=Path,required=True)
 args=parser.parse_args();source=args.analysis.resolve();mount=Path('/media/zhuyupei/SanStand')
 if not mount.is_mount()or mount not in source.parents:raise ValueError('MOUNTED_EXTERNAL_ANALYSIS_REQUIRED')
 source_hash=digest(source);report=json.loads(source.read_text())
 if not report['analysis_complete']:raise ValueError('CORE_RGB_ANALYSIS_INCOMPLETE')
 inputs=dict(report['input_sha256_by_path']);inputs[str(source)]=source_hash
 inputs[str(Path(__file__).resolve())]=digest(__file__)
 for path,sha in inputs.items():
  if digest(path)!=sha:raise ValueError('INPUT_HASH_MISMATCH: '+path)
 frames=[f for f in report['frames']if f['independent_RGB_comparison_selection']['accepted']]
 pairs=[motion_metrics(a,b)for a,b in itertools.combinations(frames,2)]
 output=dict(method='All usable pairs from independently solved RGB board poses.',
      pairs=pairs,input_sha256_by_path=inputs,limitations=report['limitations']+[
       'One completed movement without signed/orthogonal/return repeats cannot establish '
       'the unique joint,hardware root cause,or identity with prior scene errors.',
       'Multiple pair comparisons sharing a source frame are dependent,not independent replication.',
       'Near-static motion rotation has no reliable axis decomposition; translation remains board-scale dependent.'])
 for path,sha in inputs.items():
  if digest(path)!=sha:raise ValueError('INPUT_CHANGED_DURING_ANALYSIS: '+path)
 target=source.parent/'all_motion_pairs.json';target.write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
 saved=source.parent/'all_motion_pairs_source.py';saved.write_bytes(Path(__file__).read_bytes())
 (source.parent/'all_motion_pairs_sha256.json').write_text(json.dumps(
     {p.name:digest(p)for p in (target,saved)},indent=2)+'\n')
 print(json.dumps(dict(output=str(target),pairs=len(pairs)),indent=2))


if __name__=='__main__':main()
