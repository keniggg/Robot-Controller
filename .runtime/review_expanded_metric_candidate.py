#!/usr/bin/env python3
"""Independent frozen-candidate review only; no fitting, ROS, or deployment."""
import hashlib,json,os,sys,xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parent))
import audit_joint_metric_identifiability as M

ARCH=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log')
RESUME=ARCH/'grasp_repair_resume_20260928T120606Z';P=RESUME/'expanded_metric_training_once'


def rot(r):return r.as_matrix() if hasattr(r,'as_matrix') else r.as_dcm()
def frommat(r):return Rotation.from_matrix(r) if hasattr(Rotation,'from_matrix') else Rotation.from_dcm(r)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def fk_independent(xml,q,origin_offsets=None):
    robot=ET.fromstring(xml);bychild={j.find('child').get('link'):j for j in robot.findall('joint')};chain=[];child='tool0'
    while child!='base_link':
        j=bychild[child];chain.append(j);child=j.find('parent').get('link')
    T=np.eye(4)
    for j in reversed(chain):
        o=j.find('origin');xyz=np.fromstring(o.get('xyz','0 0 0'),sep=' ');rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ')
        O=np.eye(4);O[:3,:3]=rot(Rotation.from_euler('xyz',rpy));O[:3,3]=xyz
        if j.get('type')!='fixed':
            i=int(j.get('name').replace('Joint',''))-1;axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis)
            if origin_offsets is not None:
                O[:3,:3]=O[:3,:3]@rot(Rotation.from_rotvec(axis*origin_offsets[i]))
                # Exercise the URDF rpy serialization convention independently.
                angles=frommat(O[:3,:3]).as_euler('xyz')
                O[:3,:3]=rot(Rotation.from_euler('xyz',angles))
            J=np.eye(4);J[:3,:3]=rot(Rotation.from_rotvec(axis*q[i]));T=T@O@J
        else:T=T@O
    return T


def mean_transform(T):
    # SO(3) chordal rotation mean using a matrix SVD, independently of original
    # quaternion-eigenvector implementation. It is used only to freeze train Y.
    T=np.asarray(T);u,_,vt=np.linalg.svd(T[:,:3,:3].sum(0));R=u@np.diag([1,1,np.linalg.det(u@vt)])@vt
    Y=np.eye(4);Y[:3,:3]=R;Y[:3,3]=T[:,:3,3].mean(0);return Y


def main():
    if not os.path.ismount('/media/zhuyupei/SanStand'):raise RuntimeError('SanStand absent')
    d=json.loads((P/'split_declaration.json').read_text());f=json.loads((P/'fit_result.json').read_text());h=json.loads((P/'heldout_report.json').read_text());a=json.loads((P/'acceptance_summary.json').read_text())
    assert d['source_sha256']==sha(Path(__file__).with_name('fit_expanded_metric_holdout.py'))
    assert d['dependency_sha256']==sha(M.__file__)
    assert a['fit_sha256']==sha(P/'fit_result.json')
    rows,sources=M.load(RESUME/'camera_intrinsics_sensitivity')
    trainset=set(d['training_captures']);holdset=set(d['heldout_independent_captures'])
    assert len(trainset)==45 and len(holdset)==8 and not trainset&holdset
    train=[r for r in rows if r['capture'] in trainset];ind=[r for r in rows if r['capture'] in holdset]
    counts=Counter(r['group'] for r in rows if r['role']=='task_pose_validation')
    expected={r['capture'] for r in rows if r['role']=='training' or r['role']=='task_pose_validation' and counts[r['group']]>1}
    assert trainset==expected
    train_keys={(s['capture_sha256'],s['source_stamp_ns']) for s in sources if s['capture'] in trainset};hold_keys={(s['capture_sha256'],s['source_stamp_ns']) for s in sources if s['capture'] in holdset}
    assert len(train_keys)==45 and len(hold_keys)==8 and not train_keys&hold_keys
    X,xsha=M.handeye(Path('/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml'));assert xsha==d['handeye_sha256']
    z=np.array(f['fit']['z']);assert np.all(abs(z)<np.array(d['bounds_abs']));off=np.r_[0.,np.radians(z[6:]),0.]
    XX=X.copy();XX[:3,:3]=rot(Rotation.from_rotvec(np.radians(z[3:6])))@X[:3,:3];XX[:3,3]+=z[:3]*.001
    np.testing.assert_allclose(XX,a['candidate_parameters']['T_tool_camera'],atol=1e-14,rtol=0)
    bycapture={s['capture']:s for s in sources};predict=[];baseline_max=0.;candidate_max=0.;urdf_eq=0.
    for r in rows:
        xml=Path(bycapture[r['capture']]['urdf']).read_text();A=fk_independent(xml,r['q']);baseline_max=max(baseline_max,float(abs(A-r['A']).max()))
        Aq=fk_independent(xml,r['q']+off);Ac=fk_independent(xml,r['q'],off)
        candidate_max=max(candidate_max,float(abs(Aq-r['fk'](r['q']+off)).max()));urdf_eq=max(urdf_eq,float(abs(Aq-Ac).max()))
        predict.append((r,Aq@XX@r['C']))
    train22=[T for r,T in predict if r['role']=='training'];indT=np.array([T for r,T in predict if r['capture'] in holdset]);Y=mean_transform(train22)
    dp=np.linalg.norm(indT[:,:3,3]-Y[:3,3],axis=1)*1000;da=np.array([np.linalg.norm(frommat(Y[:3,:3].T@T[:3,:3]).as_rotvec())*180/np.pi for T in indT])
    independent=dict(translation_rms_mm=float(np.sqrt(np.mean(dp**2))),translation_max_mm=float(dp.max()),orientation_rms_deg=float(np.sqrt(np.mean(da**2))),orientation_max_deg=float(da.max()))
    for key,value in independent.items():np.testing.assert_allclose(value,a['independent8_fixed_training22_reference'][key],atol=1e-9,rtol=0)
    independent['threshold_pass']={key:independent[key]<=limit for key,limit in a['thresholds'].items()}
    # Pointwise plane drift plus a closed bound over each initial-plane patch:
    # original radius is XY, so Euclidean radius <= r/abs(initial normal z).
    # Rigid transport preserves that radius. n1-n0 contributes at most 2sin(theta/2)*radius.
    patches=[]
    original=json.loads((ARCH/'grasp_diagnostic_20260928T084245Z/geometry_forensics/geometry_report.json').read_text())
    original_by_key={(r['stage'],r['radius_m']):r for r in original['common_patch_comparisons']}
    for r in h['models']['expanded_45_train']['failed_grasp']:
        old=original_by_key[(r['stage'],r['radius_m'])];nz=abs(old['fits']['initial']['n'][2]);rad=r['radius_m']/nz
        bound=abs(r['later_signed_plane_distance_at_initial_anchor_mm'])+2*np.sin(np.radians(r['support_normal_change_deg'])/2)*rad*1000
        patches.append(dict(stage=r['stage'],radius_m=r['radius_m'],conservative_same_patch_local_difference_mm=float(bound),angle_deg=r['support_normal_change_deg']))
    rng=np.random.RandomState(2309);xml=Path(sources[0]['urdf']).read_text()
    for q in rng.uniform(-np.pi,np.pi,(200,6)):
        urdf_eq=max(urdf_eq,float(abs(fk_independent(xml,q+off)-fk_independent(xml,q,off)).max()))
    assert baseline_max<1e-9 and candidate_max<1e-12 and urdf_eq<2e-14
    result=dict(review_only=True,refit_performed=False,production_written=False,motion_authorized=False,reviewed_directory=str(P),
        input_sha256={n:sha(P/n) for n in ['split_declaration.json','fit_result.json','heldout_report.json','acceptance_summary.json']},
        split=dict(training=45,original_training=22,former_task_training=23,heldout_independent=8,duplicate_sources=False,train_holdout_overlap=False,
            training_groups=dict((str(k),v) for k,v in Counter(r['group'] for r in train).items()),
            development_caveat='No heldout observations enter this fit objective. Expansion was decided after observing old-model heldout failure, so reused reports are development validation, not a prospective blind test.'),
        math=dict(independent_FK_archive_max_abs=baseline_max,independent_candidate_FK_max_abs=candidate_max,origin_rotation_equivalence_max_abs=urdf_eq,
            origin_equivalence_trials=255,origin_formula='origin_new=[R_origin_old @ Exp(axis_local*delta), xyz_origin_old]; raw q, axis, limits unchanged',
            X_rotation_convention='left rotation perturbation in tool coordinates, not Euler-angle component addition',
            X_rotation_det=float(np.linalg.det(XX[:3,:3])),fixed_training_reference='SVD rotation mean and arithmetic translation of ORIGINAL22 only'),
        independent8=independent,failed_grasp_common_patch_enclosures=patches,
        max_common_patch_local_difference_bound_mm=max(r['conservative_same_patch_local_difference_mm'] for r in patches),
        max_common_patch_normal_change_deg=max(r['angle_deg'] for r in patches),
        assessment='Supports a new bounded independent physical validation with this candidate frozen; not production deployment or a claim that the physical grasp error is resolved.',
        limits=['The independent8 are archive-heldout from fitting but have been examined repeatedly during model development.',
                'Former 16/2/5 task groups are now training and cannot be used as independent confirmation.',
                'Adjacent frames within a stationary window are correlated; they are not independent sample-count gains.',
                'Same depth/PnP sensor and board metric do not independently establish physical joint zeros or absolute fingertip clearance.',
                'The common-patch bound applies to the archived visible table patches, not an unseen target or a new collision authorization.',
                'Equal hashes validate numerical model provenance, not unchanged mechanical assembly by themselves.'])
    out=Path(json.loads((Path(__file__).resolve().parent/'latest_restart_grasp.json').read_text())['directory'])/'expanded_metric_independent_review';out.mkdir(exist_ok=True)
    (out/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n');(out/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps({k:v for k,v in result.items() if k not in ['failed_grasp_common_patch_enclosures','limits']},ensure_ascii=False,indent=2))
if __name__=='__main__':main()
