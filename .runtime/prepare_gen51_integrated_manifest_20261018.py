#!/usr/bin/env python3
"""Freeze reviewed GEN51 candidate bytes for the existing guarded runner."""
import hashlib, json, os, shutil, time
from pathlib import Path
W=Path('/home/zhuyupei/alicia_wa_full/.worktrees/protocol-v3-upgrade')
E=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
V=E/'ros_log/full_latest_gen41_restart_20261018T011222Z/verification'
C=W/'.runtime/gen51_integrated_candidate'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,x):
    with p.open('x') as f:
        json.dump(x,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
assert os.path.ismount('/media/zhuyupei/SanStand')
base=W/'.runtime/gen51_baseline_source/runner_manifest.json'
assert sha(base)=='a71d4753523d9482619eb4a9825db5dca51609a36d200538f0756cef8e8a2988'
b=json.loads(base.read_text());original=b['source_sha256']
assert all(sha(W/p)==h for p,h in original.items())
prefix='src/alicia_flexible_grasp_supervisor/'
guide=prefix+'src/alicia_flexible_grasp/robot/pregrasp_compensation.py'
outer=prefix+'src/alicia_flexible_grasp/vision/outer_rgb_rectangle.py'
cuboid=prefix+'src/alicia_flexible_grasp/vision/cuboid_rgb_interval.py'
testg=prefix+'tests/test_gen51_shared_held_guide_fallback.py'
testr=prefix+'tests/test_gen51_rgb_contour_tolerance.py'
fixture=prefix+'tests/fixtures/gen51_g50_held_guide_inputs.json'
sources={guide:('gen51_held_guide_candidate','894d7c4e0d2c68feed79992218da3c184931794a7d2437d736043d09d7c262c0'),
outer:('gen51_cuboid_4px_candidate','19693a71aa3283592b0f42413b9ec254b414eb92b89b84c8777f67d32f0162c8'),
cuboid:('gen51_cuboid_4px_candidate','1199f6ad9d0f55a573e16749000a18d5c54240d81e8b92148c46877d1b45e0bb'),
testg:('gen51_held_guide_candidate','6b3587d81baab3f5cd7fd88c6636742486a51809d6f710b645c3160521ea675a'),
fixture:('gen51_held_guide_candidate','7114e60bcaa5feb6a416f57a9a04b6efa2530d8fb4d20ec148057c99244083ed')}
assert (W/'.runtime/gen51_cuboid_4px_candidate'/testr).is_file()
sources[testr]=('gen51_cuboid_4px_candidate',sha(W/'.runtime/gen51_cuboid_4px_candidate'/testr))
for p,(root,h) in sources.items(): assert sha(W/'.runtime'/root/p)==h
assert all(sha(C/p)==h for p,h in original.items())
expected=dict(original)
for p,(root,h) in sources.items():
    dest=C/p;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(W/'.runtime'/root/p,dest);assert sha(dest)==h;expected[p]=h
prod=[guide,outer,cuboid]
extra=[prefix+'tests/'+p for p in ('test_pregrasp_compensation.py','test_held_pregrasp_selection.py','test_held_guide_joint_neighborhood.py')]
focus=[prefix+'tests/'+p for p in ('test_pregrasp_feedforward.py','test_held_approach_gateway.py','test_gen45_fresh_held_path_chain.py','test_contact_sequence_precheck.py','test_gen31_outer_rgb_box.py','test_gen38_cuboid_rgb_interval.py','test_gen38_cuboid_model_contract.py','test_gen38_cuboid_silhouette.py','test_gen48_cuboid_visibility_family.py','test_gen26_inferred_box_remote_pipeline.py')]+extra+[testg,testr]
assert all(p in b['test_scope']+extra+[testg,testr] for p in focus)
m=dict(kind='GEN51_frozen_offline_candidate',workspace_root=str(C),variant='candidate',
baseline_manifest=dict(path=str(base),sha256=sha(base)),allowed_production_paths=prod,
production_source_changes=[dict(path=p,baseline_sha256=original[p],candidate_sha256=expected[p]) for p in prod],
new_production_sources=[],new_tests=[dict(path=p,sha256=expected[p]) for p in (testg,testr)],
supplemental_existing_tests=[dict(path=p,sha256=original[p]) for p in extra],
new_fixture_sources=[dict(path=fixture,sha256=expected[fixture])],source_sha256=expected,
full_scope_ready=False,required_changed_sources=prod,required_imports_by_test_file={testg:[guide],testr:[outer,cuboid]},
focus_scope=focus,created_ns=time.time_ns(),stage='isolated_frozen_for_guarded_collection_not_tested')
out=V/'gen51_integrated_collect_preparation_r1';out.mkdir()
save(out/'candidate_manifest.json',m)
save(out/'result.json',dict(manifest_path=str(out/'candidate_manifest.json'),manifest_sha256=sha(out/'candidate_manifest.json'),
source_count=len(expected),production_changes=3,new_tests=2,new_fixture=1,source_installed=False,tests_executed=False))
print(json.dumps(dict(manifest=str(out/'candidate_manifest.json'),sha256=sha(out/'candidate_manifest.json'),source_count=len(expected))))
