"""Read only sealed R30 array provenance; no qualifier, fitting, ROS or tests."""
from pathlib import Path
import gzip,hashlib,json,os,time
import numpy as np
W=Path(__file__).resolve().parents[1]
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
G=E/'grasp_records/20261018T123624.045793Z_1db3d3bb4c554d7590c8c8003c414fb5'
OUT=V/'gen51_r30_point956_origin_review_r1'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def shaped_digest(a):
    a=np.ascontiguousarray(a,dtype='<f8')
    h=hashlib.sha256();h.update(str(a.shape).encode('ascii'))
    h.update(b'\0'+a.dtype.str.encode('ascii')+b'\0');h.update(a.tobytes())
    return h.hexdigest()
def main():
    OUT.mkdir(exist_ok=False)
    audit_path=G/'keyframes_audits/3fcf580665a8226efad87b55d1a693dbfd029a7a4641ad3208ee4679d8a7b39b.json'
    assert sha(audit_path)==audit_path.stem
    a=read(audit_path);f=a['multiview_surface']['fused_scene_geometry']
    points=np.array(a['replay_geometry']['object_points_base_m'],dtype=np.float64)
    assert points.shape==(2347,3) and f['current_raw_point_count']==790 and f['fused_point_count']==1616
    assert shaped_digest(points)==f['enclosed_points_sha256']
    raw=points[:790]
    assert shaped_digest(raw)==f['current_raw_points_sha256']
    ref=f['surface_replay']['references']['snapshot'];p=Path(ref['store_root'])/ref['relative_path']
    compressed=p.read_bytes();assert sha(p)==ref['gzip_sha256']
    plain=gzip.decompress(compressed)
    assert len(plain)==ref['uncompressed_bytes'] and hashlib.sha256(plain).hexdigest()==ref['object_sha256']
    surface=json.loads(plain)
    # Content addresses preserve this receipt independently of request-local
    # GeometryEstimate, which intentionally has no point-view field.
    fused=np.asarray(surface['points_base'],dtype=np.float64)
    assert fused.shape==(1616,3) and shaped_digest(fused)==f['fused_points_sha256']
    target=points[956]
    raw_matches=np.flatnonzero(np.all(raw==target,axis=1)).tolist()
    fused_matches=np.flatnonzero(np.all(fused==target,axis=1)).tolist()
    assert raw_matches==[] and fused_matches==[171]
    origins=[dict(fused_point_index=i,view_index=surface['view_indices'][i],
        view_sources=surface['view_sources'][i],
        source_stamps_ns=[surface['view_stamps_ns'][j] for j in surface['view_sources'][i]])
        for i in fused_matches]
    assert origins[0]['view_index']==0 and origins[0]['view_sources']==[0]
    assert origins[0]['source_stamps_ns']==[1792326986987499713]
    # Reproduce only the existing order-preserving exact XYZ union. The raw
    # array is already independently pinned to the original audit digest.
    combined=np.concatenate((raw,fused),axis=0)
    _,idx=np.unique(combined,axis=0,return_index=True)
    union=combined[np.sort(idx)]
    assert np.array_equal(union,points)
    core_path=V/'gen51_retry_tabletop_review_r1/contour4px_formal_r1/qualification_core.json'
    core=read(core_path);conflict=core['audit']['measured_geometry_conflict']
    assert core['code']=='BOX_MEASURED_GEOMETRY_CONFLICT'
    assert conflict['point_index']==956 and conflict['axis_index']==0 and conflict['point_base_m']==target.tolist()
    assert conflict['excess_m']==0.001215624075010243 and conflict['permitted_observed_excess_m']==.001
    baseline=read(W/'.runtime/gen51_baseline_source/manifest.json')['source_sha256']
    sources={
      'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/fused_scene_geometry.py': '109-121 exact raw/fused union; 147-154 original corners used only for enclosure containment; 159-160 result point set is the measured union.',
      'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/regular_box_inference.py': '986-996 unchanged object_points_base passed into measured family excess; 317-345 max point/axis conflict.',
      'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py': '22116 onward serializes frozen geometry directly; per-view provenance is instead retained by the frozen surface receipt.'}
    for rel in sources:assert sha(W/rel)==baseline[rel]
    evidence={str(p):sha(p) for p in [audit_path,core_path,p,Path(__file__),
        W/'.runtime/gen51_baseline_source/manifest.json']}
    evidence.update({str(W/rel):sha(W/rel) for rel in sources})
    result=dict(kind='GEN51_R30_point956_origin_readonly_review',passed=True,created_ns=time.time_ns(),
      conclusion='Registered historical measured surface point from view 0; not present in current raw point set and not a synthetic OBB corner.',
      original_snapshot_stamp_ns=a['snapshot_stamp_ns'],point_index=956,point_base_m=target.tolist(),
      measured_point_count=len(points),measured_points_bare_float64_sha256=hashlib.sha256(points.astype('<f8').tobytes()).hexdigest(),
      measured_points_shape_dtype_bytes_sha256=shaped_digest(points),current_raw_point_count=len(raw),
      current_raw_digest_exact=True,current_raw_exact_matches=raw_matches,fused_point_count=len(fused),
      fused_point_digest_exact=True,fused_exact_matches=fused_matches,restored_view_provenance=origins,
      source_age_at_request_sec=(a['snapshot_stamp_ns']-origins[0]['source_stamps_ns'][0])/1e9,
      original_union_reproduced_exact=True,duplicates_removed=len(combined)-len(union),
      conflict_axis=0,excess_m=conflict['excess_m'],unchanged_permitted_excess_m=.001,
      evidence_sha256=evidence,source_locations=sources,
      limitations=[
        'GeometryEstimate itself lacks per-point view provenance. This attribution uses its exact union and the separately sealed registered surface points plus view_sources.',
        'The historical coordinate is in the registered current-snapshot surface frame; this does not prove that registration, the original depth, or the new RGB fit is wrong.',
        'No conclusion is drawn that the physical surface is absent from the current image/depth, that the point should be removed, or that the 1mm consistency gate should be relaxed.',
        'The 4px replay reached this unchanged consistency gate and still failed; no full Task or grasp success is claimed.'],
      no_new_algorithm=True,qualifier_or_tests_run=False,production_or_parameters_modified=False,
      ROS_initialized=False,hardware_contacted=False,temperature_read=False)
    path=OUT/'result.json'
    with path.open('x') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n');stream.flush();os.fsync(stream.fileno())
    assert read(path)==result
    print(json.dumps(dict(path=str(path),sha256=sha(path),origin=origins[0],age_sec=result['source_age_at_request_sec']),ensure_ascii=False))
if __name__=='__main__':main()
