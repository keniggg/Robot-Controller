"""Persist finite conclusions from fixed offline seed-motion controls."""
import hashlib,json
from pathlib import Path

ROOT=Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller/ros_log/regional_production_20261007T011805.778547Z/quality_seed_local_motion_risk_v1')
path=ROOT/'report.json';raw=path.read_bytes();r=json.loads(raw)
keys=[('snapshot_00.npz','bottom_strip_x'),('snapshot_21.npz','rotation_0.2'),('snapshot_05.npz','projective_x_0.006')]
examples=[]
for filename,control in keys:
    found=next(x for x in r['regressions'] if x['file']==filename and x['control']['name']==control)
    examples.append(found)
out=dict(schema=1,diagnostic_only=True,production_modified=False,motion_authorized=False,
    report_sha256=hashlib.sha256(raw).hexdigest(),source_snapshots=32,unique_anchor_region_groups=19,
    cases=475,identity_controls=19,nonidentity_controls=456,
    baseline_motion_reject_to_high_quality_pair_accept=78,regression_counts=r['regression_counts'],
    representative_counterexamples=examples,
    figure_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(ROOT.glob('*.png'))},
    verdict='Do not directly promote fixed qualityLevel0.12: the actual anchor controls demonstrate additional local/rotational/projective motion blind spots with all original gates retained.',
    findings=[
        'The original0.02 seed was reconstructed bitwise on all32 snapshots; repeated identical full raw anchors were deduplicated byregion+SHA. Every one of19 identity controls passes both seed qualities.',
        'Each anchor receives the same25 predeclared controls: identity,16 fixed local0.8px controls,4 rotations,4 perspective controls. Generation never uses measured flow or an observed rejection to select points, masks or parameters.',
        'Snapshot00 bottom-strip motion:0.02 has64 valid points including10 in the moving area and P90~0.736px.0.12 has45 valid points with only1 in that area and P90=0. Both retain4 covered quadrants.22.44% ofROI pixels have commanded motion>0.25px.',
        'Snapshot21 rotation+0.2degrees:0.02 P90~0.294px rejects,0.12 P90~0.234px accepts.56.21% ofROI pixels have known geometric movement>0.25px; high-quality points concentrate nearer the rotation center.',
        'Snapshot05 centered weak perspective:0.02 P90~0.365px rejects,0.12 P90~0.244px accepts while44.25% ofROI pixels move>0.25px.',
        'Maintaining4 quadrant labels is weaker than maintaining coverage within quadrants: the bottom-strip example loses occupied4x4 cells13->9; nested rotation21 loses13->8. Strong feature concentration can hide weaker-texture peripheral movement.',
        'Previously passing whole-image translations cannot expose this failure mode because every retained strong feature moves in those controls.'
    ],
    limitations=[
        '78 is a count in a bounded designed counterexample set, not an estimated real-world false-accept rate.',
        'Synthetic interpolation and local smooth backward remapping are not a calibrated physical motion experiment; global rotations and perspective controls provide separate geometric examples.',
        'Original minimum24 seeds,>=80% tracked fraction,>=3 covered quadrants, forward/backward gate and0.25px P90 remain unchanged. Only the offline seed-quality input differs.',
        'No fresh0.35s temporal window, RGB-D support-plane fit or grasp task has been validated here. No production parameter was changed.',
        'Controlled unobstructed illumination-stable real data is still required to isolate the original visual-motion cause.'
    ])
assert path.read_bytes()==raw and Path('/media/zhuyupei/SanStand').is_mount()
p=ROOT/'interpretation.json'
with p.open('x')as f:json.dump(out,f,ensure_ascii=False,indent=2,allow_nan=False);f.write('\n')
print(json.dumps(dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())))
