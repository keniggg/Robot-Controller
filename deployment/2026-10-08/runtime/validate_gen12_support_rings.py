"""Run the actual support audit/refit on eight cached native measured frames."""
import argparse,hashlib,json
from pathlib import Path
from types import SimpleNamespace
from dataclasses import asdict
import numpy as np,rosbag,rospy
from alicia_grasp_modes.tabletop import Config,_audit_current_support,segment

BASE=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
RECORD=BASE/'grasp_records/20261006T074059.722455Z_4f669c3e99b047e69ee49b6a8966a8ab'
CACHE=BASE/'ros_log/grasp_supplement_view_20261006T074059Z/support_fit_diagnostic_v1'

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',required=True);args=parser.parse_args()
    assert Path('/media/zhuyupei/SanStand').is_mount()
    m=json.loads((RECORD/'manifest.json').read_text());assert m['state']=='complete' and m['recorder_closed'] is True
    old=json.loads((CACHE/'report.json').read_text());camera=json.loads((CACHE/'camera_metadata_parameters.json').read_text())
    with rosbag.Bag(str(RECORD/'keyframes.bag'),'r') as b:
        s=next(json.loads(message.data) for _,message,_ in b.read_messages(
            topics=['/perception/unknown/detector_status'],start_time=rospy.Time.from_sec(1791272468.),end_time=rospy.Time.from_sec(1791272468.3))
            if json.loads(message.data).get('source_stamp_ns')==old['rows'][0]['source_ns'])
    anchor=SimpleNamespace(position_base=np.array(s['anchor_position_base']),support_footprint_base=np.array(s['reference_support_footprint_base']))
    plane=np.array(s['reference_support_plane_base']);K=[camera[k] for k in ('fx','fy','cx','cy')]
    cfg=Config();rows=[]
    for prior in old['rows']:
        data=np.load(CACHE/('%d.npz'%prior['source_ns']));d=data['depth_m'];F=data['T_base_camera']
        assert np.array_equal(plane,np.array(prior['reference_plane_base']))
        audit=_audit_current_support(d,K,F,cfg,anchor,plane)
        refit=segment(d,K,F,cfg,target_position_base=anchor.position_base,plane_base=plane,
                      target_footprint_base=anchor.support_footprint_base)
        assert audit['support_current_audit_available'] is True
        assert audit['support_current_audit_quadrants']>=3 and audit['support_current_audit_inlier_ratio']>=cfg.support_min_ratio
        assert audit['support_reference_max_distance_m']<=cfg.support_max_local_shift_m
        assert refit.metrics['support_reference_max_distance_m']<=cfg.support_max_local_shift_m
        row={'source_ns':prior['source_ns'],'old_detail':prior['detail'],'audit':audit,'refit_metrics':refit.metrics,
             'refit_candidate_count':len(refit.candidates),'geometry_or_motion_authority':False}
        rows.append(row);print(json.dumps({'source_ns':prior['source_ns'],'audit':audit}),flush=True)
    source=Path('src/alicia_grasp_modes/src/alicia_grasp_modes/tabletop.py')
    result={'passed':True,'scope':'offline native support audit/refit only; no RGB identity or real-time deadline claim',
            'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'diagnostic_source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'original_diagnostic_report_sha256':hashlib.sha256((CACHE/'report.json').read_bytes()).hexdigest(),
            'config':asdict(cfg),'configuration_provenance':'original ready-frame audit reproduced exactly using Config defaults; later post-attempt unknown_perception snapshot is empty; source initialization reads once',
            'anchor':anchor.position_base.tolist(),'reference_support_footprint_base':anchor.support_footprint_base.tolist(),
            'rows':rows}
    out=Path(args.out);out.mkdir(parents=True,exist_ok=False);(out/'validation.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')

if __name__=='__main__':main()
