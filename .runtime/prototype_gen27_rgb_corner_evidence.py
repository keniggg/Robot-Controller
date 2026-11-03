"""Offline hypothesis: qualify corner junctions from actual RGB edge pixels.

No inferred rectangle is painted into a mask. The depth, mask-boundary,
occupancy, hole, outside-continuation and uncertainty checks remain intact.
This is a diagnostic candidate policy, not a production qualification result.
"""
import inspect
import json
from pathlib import Path
import time

import numpy as np
import prototype_gen27_rgb_search_domain as search

base, box = search.base, search.box
output = base.INPUT.parent / 'gen27_rgb_corner_evidence_prototype_r1'
assert Path('/media/zhuyupei/SanStand').is_mount()
assert not output.exists()
output.mkdir()
input_hashes = {}
reports = []

for radius in (2, 4, 6):
    search.install_offline_search(radius)
    original_visual, _locations = search.clone_search(radius)
    source = inspect.getsource(box._visual_rectangle)
    source = source.replace('np.ones((5,5), np.uint8)) == 0',
                            'np.ones((_search_diameter,_search_diameter), np.uint8)) == 0')
    old = '''        for corner in projected:
            x,y=np.rint(corner).astype(int)
            if min(x,y,depth.shape[1]-1-x,depth.shape[0]-1-y)<4:
                good_candidate=False;break
            patch=mask[y-2:y+3,x-2:x+3]
            # A true convex rectangle corner occupies one quadrant of its
            # centered patch; expecting half/full occupancy rejects the very
            # corners this visual branch is intended to measure.
            corner_cov.append(float(min(1.,np.mean(patch)/.25)))
'''
    new = '''        corner_edges=cv2.dilate(edges,np.ones((3,3),np.uint8))>0
        for corner_index,corner in enumerate(projected):
            x,y=np.rint(corner).astype(int)
            if min(x,y,depth.shape[1]-1-x,depth.shape[0]-1-y)<4:
                good_candidate=False;break
            # Both real branches must approach this corner. Long lines with
            # an unsupported extrapolated intersection do not suffice.
            branch_support=[]
            for neighbor in ((corner_index-1)%4,(corner_index+1)%4):
                ray=projected[neighbor]-corner
                length=np.linalg.norm(ray)
                distances=np.linspace(0.,min(8.,.18*length),9)
                samples=np.rint(corner+np.outer(distances,ray/length)).astype(int)
                branch_support.append(float(np.mean(corner_edges[samples[:,1],samples[:,0]])))
            # Preserve the qualification field's half-coverage admission;
            # the separate exact gate requires 7/9 samples in EACH branch.
            corner_cov.append(min(branch_support))
            if min(branch_support)<7./9.:
                good_candidate=False;break
'''
    assert source.count(old) == 1
    source = source.replace(old, new)
    namespace = dict(vars(box), _search_diameter=2*radius+1)
    exec(compile(source, '<offline_actual_rgb_corner>', 'exec'), namespace)
    visual_function = namespace['_visual_rectangle']

    def make_qualifier():
        context = {}
        namespace = dict(vars(box), _prototype_axes=None)

        def visual(*args):
            context['args'] = args
            started = time.perf_counter()
            value = visual_function(*args)
            context['visual'] = value
            context['visual_seconds'] = time.perf_counter()-started
            return value

        source = inspect.getsource(box.qualify_regular_box_top)
        old = '        coord = top @ axes[:, :2]\n'
        assert source.count(old) == 1
        source = source.replace(old, '        if _prototype_axes is not None:\n            axes = _prototype_axes.copy()\n'+old)
        namespace['_visual_rectangle'] = visual
        exec(compile(source, '<offline_actual_rgb_corner_qualifier>', 'exec'), namespace)
        return namespace, context, namespace['qualify_regular_box_top']

    base.make_qualifier = make_qualifier
    specs = [
        ('initial_raw', 'source_1792050907639824151.npz', 'geometry_1792050907639824151.json'),
        ('initial_fused', 'initial_three_frame_reconstruction.npz', 'initial_three_frame_geometry.json'),
        ('nearfield_raw', 'source_1792050956912783622.npz', 'geometry_1792050956912783622.json'),
    ]
    for name, raw, geometry in specs:
        paths = [base.INPUT/raw, base.INPUT/geometry]
        input_hashes.update({str(path): base.digest(path) for path in paths})
        report = base.qualify_source(*paths,
            'offline_rgb_corner_policy_hypothesis_not_deployed',
            ['Actual RGB corner branches replace segmentation mask occupancy only for the visual corner check.',
             'Fusion reconstruction is not a retained original production qualification audit.',
             'Passing a model is not aperture, collision, plan or physical grasp success.'])
        target = output / ('%s_radius%d.json' % (name, radius))
        target.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
        row = dict(name=name, radius=radius, path=str(target), sha256=base.digest(target),
            actual_direction_count=report['candidate_count'],
            visual_rectangle_found_count=sum(r['visual_rectangle_found'] for r in report['candidates']),
            qualified_count=report['qualified_rgb_candidate_count'],
            selected_model=report['selected_model'])
        reports.append(row)
        small = {key: value for key,value in row.items() if key!='selected_model'}
        if row['selected_model']:
            small.update(size=row['selected_model']['size_xyz_m'],
                edge_uncertainty=row['selected_model']['empirical_edge_uncertainty_m'])
        print(json.dumps(small), flush=True)

assert all(base.digest(path)==digest for path,digest in input_hashes.items())
receipt = dict(classification='offline_corner_evidence_hypothesis', production_changes=False,
    no_ros_or_hardware=True, input_hashes=input_hashes,
    script_sha256=base.digest(Path(__file__)), results=reports,
    policy='Both actual Canny corner branches need at least 7 of 9 samples within 1 pixel; no mask occupancy is treated as RGB corner evidence.')
(output/'receipt.json').write_text(json.dumps(receipt, indent=2, ensure_ascii=False)+'\n')
print(json.dumps(dict(receipt=str(output/'receipt.json'),sha256=base.digest(output/'receipt.json'))))
