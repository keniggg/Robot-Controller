"""Classify the original 20260910 RGBD regression under open-contact semantics."""
import ast
from collections import Counter
from collections.abc import Mapping
from dataclasses import asdict
import importlib.util
import json
import os
from pathlib import Path
import platform
import signal
import struct
import sys
import time

import extract_gen50_retry_r30_rgb_20261018 as boundary

W = Path(__file__).resolve().parents[1]
C = W / '.runtime/gen54_integrated_candidate'
OUT = boundary.N / 'verification/gen54_geometry_pipeline_r1/recorded_local_midpoints_r1'


def main():
    assert boundary.E.parent.parent.is_mount() and not OUT.exists()
    OUT.mkdir()
    os.environ.update(PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
                      ROS_MASTER_URI='http://127.0.0.1:1', ROS_LOG_DIR=str(OUT),
                      ROS_HOME=str(OUT), TMPDIR=str(OUT))
    sys.dont_write_bytecode = True
    native = os.uname()
    platform.uname = lambda: platform.uname_result(*(tuple(native) + (native.machine,)))
    platform.architecture = lambda *a, **k: (str(8 * struct.calcsize('P')) + 'bit', 'ELF')
    boundary.OUT = OUT
    sys.addaudithook(boundary.audit)
    signal.signal(signal.SIGALRM, lambda *a: (_ for _ in ()).throw(TimeoutError('90 second replay budget')))
    signal.alarm(90)
    paths = [C / 'src/alicia_flexible_grasp_supervisor', C / 'src/alicia_flexible_grasp_supervisor/src',
             C / 'src/alicia_grasp_modes/src', W / 'devel/lib/python3/dist-packages',
             Path('/opt/ros/noetic/lib/python3/dist-packages')]
    sys.path[:0] = list(map(str, paths))
    source = C / 'src/alicia_flexible_grasp_supervisor/tests/test_remote_grasp6d_streaming.py'
    spec = importlib.util.spec_from_file_location('gen54_recorded_compat_test', source)
    test = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(test)
    from alicia_flexible_grasp.grasp import gripper_geometry as gg
    from alicia_flexible_grasp.grasp import tabletop_geometry_candidates as tg
    from alicia_flexible_grasp.vision.cuboid_rgb_interval import _plain
    remote = test.remote_node
    cls = remote.RemoteGrasp6DNode
    original_physical = gg._evaluate_physical_candidate
    tree = ast.parse(Path(gg.__file__).read_text())
    fn = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == '_evaluate_physical_candidate')
    for statement in fn.body:
        if isinstance(statement, ast.Assign):
            name = next((x.id for x in statement.targets if isinstance(x, ast.Name)), None)
            if name in ('endpoint_contact', 'segment_contact'):
                values = (False, False, True, True) if name == 'endpoint_contact' else (False, True, True)
                statement.value = ast.Tuple(elts=[ast.Constant(v) for v in values], ctx=ast.Load())
    ast.fix_missing_locations(fn)
    env = dict(gg.__dict__)
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(gg.__file__), 'exec'), env)
    old_physical = env['_evaluate_physical_candidate']
    real_generate = cls._generate_tabletop_candidates
    real_normalize = cls._normalize_tabletop_candidate
    real_check = gg._check_boxes
    outputs = []
    started = time.monotonic()
    for label, physical in [('diagnostic_original_adjacent_midpoints_80_to_95', original_physical)]:
        gg._evaluate_physical_candidate = physical
        rows = []
        generated = {}
        checks = []
        context = {}
        def generate(self, geometry, *args, **kwargs):
            kwargs['refine_obb_neighbor_angles'] = False
            original_angles=tg._sample_angles
            def local_angles(rotation, normal, config, refine_neighbors=False):
                bx=tg._plane_basis_x(normal);by=test.np.cross(normal,bx)
                return tuple((angle,bx*test.np.cos(test.np.deg2rad(angle))+by*test.np.sin(test.np.deg2rad(angle)))
                             for angle in (82.5,87.5,92.5))
            tg._sample_angles=local_angles
            try:
                items, diagnostics = real_generate(self, geometry, *args, **kwargs)
            finally:
                tg._sample_angles=original_angles
            generated.update(count=len(items), diagnostics=diagnostics)
            context.update(node=self, geometry=geometry, items=items)
            return items, diagnostics
        def check(*args, **kwargs):
            result = real_check(*args, **kwargs)
            if not result[0]:
                checks.append(dict(result=result, transform=args[0], physical_gap_m=args[2],
                                   support_normal=args[5], support_offset=args[6],
                                   obb_center=args[7], obb_rotation=args[8], obb_size=args[9],
                                   allow_finger_contact=args[10]))
            return result
        def normalize(self, prepared, candidate):
            start = len(checks)
            row = dict(source_index=candidate.source_index, variant_index=candidate.variant_index,
                       audit=candidate.audit, required_open_width_m=candidate.required_open_width_m,
                       T_base_tool0=candidate.T_base_tool0)
            try:
                normalized = real_normalize(self, prepared, candidate)
                row.update(ok=True, gate=asdict(normalized.geometry_gate))
                return normalized
            except remote.CandidateContractError as exc:
                row.update(ok=False, error=str(exc))
                raise
            finally:
                row['failing_checks'] = checks[start:]
                rows.append(row)
        cls._generate_tabletop_candidates = generate
        cls._normalize_tabletop_candidate = normalize
        gg._check_boxes = check
        # The compiled diagnostic function owns a distinct namespace.
        env['_check_boxes'] = check
        caught = ''
        try:
            test.test_configured_resolution_preserves_recorded_bilateral_contact()
        except AssertionError as exc:
            caught = str(exc)[:500]
        finally:
            cls._generate_tabletop_candidates = real_generate
            cls._normalize_tabletop_candidate = real_normalize
            gg._check_boxes = real_check
        node=context['node'];geometry=context['geometry'];surface, reference=node._snapshot_multiview_surface()
        np=test.np
        normal=geometry.support_normal_base;bx=tg._plane_basis_x(normal);by=np.cross(normal,bx)
        directions=[]
        for angle in generated['diagnostics']['sampled_angles_deg']:
            axis=bx*np.cos(np.deg2rad(angle))+by*np.sin(np.deg2rad(angle))
            evidence,bounds=gg.evaluate_bilateral_surface_evidence_and_bounds(surface,geometry.center_base,
                axis,-normal,node.gripper_geometry,12,support_normal_base=normal)
            projection=float(np.abs(geometry.axes_base.T@axis)@geometry.size_xyz_m)
            directions.append(dict(angle_deg=angle,evidence=asdict(evidence),bounds=bounds,
                                   projection_m=projection,open_padded_projection_margin_m=.05-.0012-projection))
        outputs.append(dict(label=label, assertion=caught, accepted=sum(r['ok'] for r in rows),
                            error_counts=dict(Counter(r.get('error', '') for r in rows if not r['ok'])),
                            generated=generated, rows=rows, geometry=asdict(context['geometry']),
                            measured_original_direction_facts=directions))
    gg._evaluate_physical_candidate = original_physical
    result = dict(diagnostic_only=True, code_modified_in_memory_only=True,
                  source_path=str(source), source_sha256=boundary.sha(source),
                  fixture_sha256=boundary.sha(source.parent / 'fixtures/measured_contact_rgbd_20260910.npz'),
                  elapsed_sec=time.monotonic()-started, outputs=outputs,
                  boundary_attempts=boundary.BOUNDARY, ROS_initialized=False, hardware_contacted=False,
                  real_grasp_success=False, motion_authorized=False)
    def clean(value):
        if isinstance(value, Mapping):
            return {str(k): clean(v) for k, v in value.items()}
        if isinstance(value, (tuple, list)):
            return [clean(v) for v in value]
        return _plain(value)
    (OUT / 'result.json').write_text(json.dumps(clean(result), indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(result=str(OUT / 'result.json'),
                         outputs=[dict(label=x['label'], accepted=x['accepted'], errors=x['error_counts']) for x in outputs])))
    signal.alarm(0)


if __name__ == '__main__':
    main()
