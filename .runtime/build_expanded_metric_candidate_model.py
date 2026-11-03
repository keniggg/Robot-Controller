#!/usr/bin/env python3
"""Build a review-only URDF/X bundle from the frozen 45-frame candidate.

No ROS connection, deployment, or motor commands. Only four origin rpy fields
are rewritten. Existing accepted/command angles, limits, meshes, and axes stay.
"""
import argparse
from datetime import datetime,timezone
import hashlib,itertools,json,os,re,subprocess,warnings
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
import audit_joint_metric_identifiability as M
import fit_expanded_metric_holdout as E
import evaluate_frozen_metric_failed_grasp as F
from alicia_flexible_grasp.robot.observation_path_guard import SerialUrdfFk


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rpy_matrix(v):
    return M.rotation([0,0,1],v[2])@M.rotation([0,1,0],v[1])@M.rotation([1,0,0],v[0])


def change_origin(xml,name,new_value):
    pattern=re.compile(r'<joint\b[^>]*\bname="'+re.escape(name)+r'"[^>]*>.*?</joint>',re.S)
    matches=list(pattern.finditer(xml))
    if len(matches)!=1:raise ValueError('unique named joint block required: '+name)
    match=matches[0];block=match.group();origin=re.compile(r'(<origin\b[^>]*\brpy=")([^"]*)(")')
    values=list(origin.finditer(block))
    if len(values)!=1:raise ValueError('one explicit rpy origin required')
    old=values[0].group(2);block,count=origin.subn(lambda m:m.group(1)+new_value+m.group(3),block)
    assert count==1
    return xml[:match.start()]+block+xml[match.end():],old


def main():
    p=argparse.ArgumentParser();p.add_argument('fit_directory',type=Path);a=p.parse_args();out=a.fit_directory/'candidate_model'
    if not os.path.ismount(F.MOUNT) or F.MOUNT not in out.resolve().parents:raise ValueError('SanStand output required; no fallback')
    if out.exists():raise ValueError('candidate bundle already exists; refusing unreviewed overwrite')
    declaration_path=a.fit_directory/'split_declaration.json';fit_path=a.fit_directory/'fit_result.json';acceptance_path=a.fit_directory/'acceptance_summary.json'
    declaration=json.loads(declaration_path.read_text());fit=json.loads(fit_path.read_text());acceptance=json.loads(acceptance_path.read_text())
    if acceptance['fit_sha256']!=sha(fit_path):raise ValueError('acceptance/fit digest mismatch')
    original_urdf=Path(declaration['all_original_sources'][0]['urdf'])
    original_handeye=Path('/home/zhuyupei/.ros/easy_handeye/d405_cc200_auto_20260927_validated_eye_on_hand.yaml')
    if sha(original_urdf)!=declaration['urdf_sha256'] or sha(original_handeye)!=declaration['handeye_sha256']:raise ValueError('original model freeze mismatch')
    xml=original_urdf.read_text();X,_=M.handeye(original_handeye);z=np.asarray(fit['fit']['z']);XX=E.modified_X(X,z)
    np.testing.assert_allclose(XX,acceptance['candidate_parameters']['T_tool_camera'],atol=1e-14,rtol=0)
    offsets=np.r_[0.,np.radians(z[6:]),0.];root=ET.fromstring(xml);joints={j.get('name'):j for j in root.findall('joint')}
    revised=xml;changes=[]
    for i in range(1,5):
        name='Joint%d'%(i+1);j=joints[name];origin=j.find('origin');old_text=origin.get('rpy');old=np.fromstring(old_text,sep=' ')
        axis=np.fromstring(j.find('axis').get('xyz'),sep=' ');axis/=np.linalg.norm(axis)
        Rold=rpy_matrix(old);Rnew=Rold@M.rotation(axis,float(offsets[i]))
        with warnings.catch_warnings():
            warnings.simplefilter('error');new=M.from_matrix(Rnew).as_euler('xyz')
        np.testing.assert_allclose(rpy_matrix(new),Rnew,atol=2e-14,rtol=0)
        new_text=' '.join(format(float(v),'.17g') for v in new)
        revised,original_text=change_origin(revised,name,new_text);assert original_text==old_text
        changes.append(dict(joint=name,axis=j.find('axis').get('xyz'),offset_rad=float(offsets[i]),offset_deg=float(np.degrees(offsets[i])),
            old_rpy=old_text,new_rpy=new_text,old_origin_rotation=Rold.tolist(),new_origin_rotation=Rnew.tolist(),origin_xyz_unchanged=origin.get('xyz')))
    # Exact byte-preservation of everything except the four origin rpy values.
    reverse=revised
    for c in changes:
        reverse,old=change_origin(reverse,c['joint'],c['old_rpy']);assert old==c['new_rpy']
    if reverse!=xml:raise ValueError('unexpected URDF edit beyond 4 rpy values')
    ET.fromstring(revised)
    names=tuple('Joint%d'%i for i in range(1,7));old_fk=M.MetricFk(xml);new_fk=M.MetricFk(revised)
    prod_old=SerialUrdfFk(xml,names);prod_new=SerialUrdfFk(revised,names)
    np.testing.assert_array_equal(prod_old.limits,prod_new.limits)
    if not np.all(np.isfinite(prod_new.limits)):raise ValueError('finite original legal ranges required')
    lo,hi=prod_new.limits.T;middle=(lo+hi)/2;rng=np.random.default_rng(20260929)
    samples=[*rng.uniform(lo,hi,size=(1024,6)),*np.asarray(list(itertools.product(*zip(lo,hi)))),middle,np.clip(np.zeros(6),lo,hi)]
    for i in range(6):
        for v in (lo[i],hi[i],np.nextafter(lo[i],hi[i]),np.nextafter(hi[i],lo[i])):
            q=middle.copy();q[i]=v;samples.append(q)
    max_tool=0.;max_camera=0.;max_rotation=0.;max_production=0.;overlap_count=0
    for q in samples:
        assert np.all(q>=lo) and np.all(q<=hi)
        expected=old_fk(q+offsets);actual=new_fk(q);production=prod_new(q)
        np.testing.assert_allclose(actual,expected,atol=5e-13,rtol=0)
        np.testing.assert_allclose(production,expected,atol=5e-13,rtol=0)
        np.testing.assert_allclose(prod_old(q),old_fk(q),atol=5e-13,rtol=0)
        camera=production@XX;wanted=expected@XX;np.testing.assert_allclose(camera,wanted,atol=5e-13,rtol=0)
        max_tool=max(max_tool,float(np.linalg.norm(actual[:3,3]-expected[:3,3])))
        max_camera=max(max_camera,float(np.linalg.norm(camera[:3,3]-wanted[:3,3])))
        max_rotation=max(max_rotation,float(np.linalg.norm(M.from_matrix(expected[:3,:3].T@actual[:3,:3]).as_rotvec())))
        max_production=max(max_production,float(np.max(np.abs(production-expected))))
        if np.all(q+offsets>=lo) and np.all(q+offsets<=hi):
            np.testing.assert_allclose(prod_old(q+offsets),production,atol=5e-13,rtol=0);overlap_count+=1
    out.mkdir(parents=True);rollback=out/'rollback_source';rollback.mkdir()
    urdf_path=out/'alicia_expanded_metric_candidate.urdf';urdf_path.write_text(revised)
    candidate_yaml=yaml.safe_load(original_handeye.read_text());t=candidate_yaml['transformation'];quat=M.from_matrix(XX[:3,:3]).as_quat();quat=quat if quat[3]>=0 else -quat
    for k,v in zip(('x','y','z'),XX[:3,3]):t[k]=float(v)
    for k,v in zip(('qx','qy','qz','qw'),quat):t[k]=float(v)
    yaml_path=out/'d405_expanded_metric_candidate_eye_on_hand.yaml'
    yaml_path.write_text('# REVIEW-ONLY CANDIDATE: not deployed; use only with bundled calibrated URDF.\n'+yaml.safe_dump(candidate_yaml,sort_keys=False))
    Xread,_=M.handeye(yaml_path);np.testing.assert_allclose(Xread,XX,atol=1e-14,rtol=0)
    assert candidate_yaml['parameters']==yaml.safe_load(original_handeye.read_text())['parameters']
    (rollback/'original_robot_description.urdf').write_bytes(original_urdf.read_bytes())
    (rollback/'original_validated_handeye.yaml').write_bytes(original_handeye.read_bytes())
    original_production_files=[Path('src/alicia_flexible_grasp_supervisor/config/handeye.yaml'),Path('src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml')]
    prior={str(p.resolve()):sha(p) for p in original_production_files}
    checked=subprocess.run(['/usr/bin/check_urdf',str(urdf_path)],capture_output=True,text=True,timeout=15)
    (out/'check_urdf.log').write_text(checked.stdout+checked.stderr)
    if checked.returncode!=0:raise RuntimeError('check_urdf rejected candidate: see candidate output log')
    verification=dict(legal_q_sample_count=len(samples),random_seed=20260929,random_interior_count=1024,all_64_limit_vertices_included=True,
        original_limits_unchanged=True,old_fk_shifted_within_original_limits_count=overlap_count,
        shifted_reference_outside_old_angle_limits_count=len(samples)-overlap_count,
        shifted_reference_note='q remains legal under unchanged original command limits. FK_original(q+offset) is mathematical reference; shifted coordinates do not redefine command limits.',
        maximum_tool_translation_error_m=max_tool,maximum_camera_translation_error_m=max_camera,maximum_tool_rotation_error_rad=max_rotation,
        maximum_production_FK_matrix_error=max_production,elementtree_parse_passed=True,production_SerialUrdfFk_parse_passed=True,
        scipy_MetricFk_parse_passed=True,check_urdf_returncode=checked.returncode,yaml_rigid_transform_roundtrip_passed=True,
        exact_reversible_four_rpy_replacements_only=True,all_axes_limits_link_names_meshes_and_other_bytes_preserved=True)
    manifest=dict(schema=1,created_utc=datetime.now(timezone.utc).isoformat(),purpose='review-only candidate bundle for prospective independent raw-data validation',
        deployed=False,ros_parameters_written=False,motor_commands_sent=0,production_URDF_modified=False,
        generator_sha256=sha(__file__),generator_path=str(Path(__file__).resolve()),frozen_fit_path=str(fit_path),frozen_fit_sha256=sha(fit_path),
        split_declaration_sha256=sha(declaration_path),acceptance_summary_sha256=sha(acceptance_path),
        original_urdf_path=str(original_urdf),original_urdf_sha256=sha(original_urdf),original_handeye_path=str(original_handeye),original_handeye_sha256=sha(original_handeye),
        candidate_urdf_path=str(urdf_path),candidate_model_sha256=sha(urdf_path),candidate_handeye_path=str(yaml_path),candidate_handeye_sha256=sha(yaml_path),
        candidate_pair_sha256=hashlib.sha256((sha(urdf_path)+':'+sha(yaml_path)).encode()).hexdigest(),
        frames=dict(base='base_link',tool='tool0',camera='camera_link'),candidate_parameters=acceptance['candidate_parameters'],origin_changes=changes,
        algebra='Torigin_new = Torigin_old * R_axis(delta). Since R_axis(delta) * R_axis(q) = R_axis(q+delta), every downstream chain transform equals original FK(q+offset).',
        command_contract='Joint names, q commands/feedback, axes, limits, origin translations, meshes and remaining XML bytes are unchanged; never send calibration offsets as additional motor motion.',
        model_cache_contract='All planners, FK guards, scene/trajectory caches and model_sha256-bound receipts must use this new URDF digest as a distinct model if explicitly deployed later. No old model cache/plan may be relabeled/reused. This generator changes no cache or process.',
        rollback_sources=[dict(copy=str(rollback/'original_robot_description.urdf'),sha256=sha(original_urdf)),dict(copy=str(rollback/'original_validated_handeye.yaml'),sha256=sha(original_handeye))],
        production_configuration_readonly_hashes=prior,verification=verification,
        prospective_validation_status='Pending fresh raw observations collected under the original running model; old logs are model-development validation, not a new untouched prospective holdout.',
        deployment_authorized_by_this_artifact=False)
    (out/'verification.json').write_text(json.dumps(verification,indent=2)+'\n');(out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    for path,digest in prior.items():assert sha(path)==digest
    assert sha(original_urdf)==declaration['urdf_sha256'] and sha(original_handeye)==declaration['handeye_sha256']
    (out/'README.md').write_text('# Review-only expanded metric model\n\nNo deployment or ROS writes performed. Pair the candidate URDF with its candidate X; do not mix with original X or apply zero offsets as motor commands.\n\nOnly Joint2–5 origin rpy values differ. Unchanged command coordinates and joint limits remain authoritative. All other URDF bytes, including axes and meshes, are preserved. `manifest.json` pins originals, candidate files, frozen fit, rollback copies, and required new model/cache identity.\n\n`verification.json` includes 1,114 legal joint configurations (1,024 deterministic random points, all 64 limit vertices, mid/zero and boundary probes) and independent production SerialUrdfFk checks. `check_urdf.log` is the standalone parser result.\n\nCandidate remains pending a new prospective validation under the original running model. Historical logs informed model development and do not count as new untouched validation.\n')
    print(json.dumps(dict(candidate_directory=str(out),candidate_model_sha256=manifest['candidate_model_sha256'],candidate_handeye_sha256=manifest['candidate_handeye_sha256'],verification=verification,origin_changes=changes),ensure_ascii=False))

if __name__=='__main__':main()
