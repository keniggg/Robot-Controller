from pathlib import Path
import json,shutil
root=Path.cwd();dep=Path(json.load(open('.runtime/current_handeye_deployment.json'))['directory'])
g=root/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/object_geometry.py'
r=root/'src/alicia_flexible_grasp_supervisor/scripts/remote_grasp6d_node.py'
for p in (g,r):shutil.copy2(p,dep/(p.stem+'_before_support_height_planning.py'))
s=g.read_text();needle='def _fit_support_plane(points, threshold_m, min_support_points):'
helper='''def _support_context_mask(depth, foreground, bbox, intrinsics, depth_scale,
                          expand_ratio, minimum_span_m):
    """Observe the gripper footprint even when the target spans few pixels.

    A tight instance mask is not negative evidence for the remainder of its
    bounding box: filtered rim pixels and mask holes can still be the object.
    With a physical footprint requested, exclude that whole uncertain box.
    This expands measured context only; the plane thresholds are unchanged.
    """
    expanded, box = _expanded_bbox_mask(depth.shape, bbox, expand_ratio)
    if isinstance(minimum_span_m, (bool, np.bool_)):
        raise ValueError('support minimum span must be numeric')
    minimum_span_m = float(minimum_span_m)
    if not np.isfinite(minimum_span_m) or minimum_span_m < 0.:
        raise ValueError('support minimum span must be finite and nonnegative')
    if minimum_span_m == 0.:
        return expanded & ~foreground
    values = np.asarray(depth)[box & np.isfinite(depth) & (depth > 0)]
    if not len(values):
        raise ValueError('support context lacks a measured target-range scale')
    range_m = float(np.median(values)) * float(depth_scale)
    if not np.isfinite(range_m) or range_m <= 0.:
        raise ValueError('support context target range is invalid')
    width, height, fx, fy, _cx, _cy = _intrinsic_values(intrinsics, depth.shape)
    x, y, bw, bh = [int(v) for v in bbox]
    # Preserve the configured context, and add the measured physical footprint.
    px = max(bw, int(np.ceil(minimum_span_m * fx / range_m)))
    py = max(bh, int(np.ceil(minimum_span_m * fy / range_m)))
    x0, y0 = int(np.floor(x + bw / 2. - px / 2.)), int(np.floor(y + bh / 2. - py / 2.))
    x1, y1 = int(np.ceil(x + bw / 2. + px / 2.)), int(np.ceil(y + bh / 2. + py / 2.))
    expanded[max(0,y0):min(height,y1), max(0,x0):min(width,x1)] = True
    return expanded & ~foreground & ~box


'''
assert needle in s;s=s.replace(needle,helper+needle,1)
needle='    outlier_std_ratio=2.0,\n):\n    """Return GeometryEstimate'
assert needle in s;s=s.replace(needle,'    outlier_std_ratio=2.0,\n    support_minimum_span_m=0.0,\n):\n    """Return GeometryEstimate',1)
needle='        support_mask = expanded & ~foreground\n';assert s.count(needle)==1
s=s.replace(needle,'''        support_mask = _support_context_mask(
            depth, foreground, bbox, intrinsics, scale,
            support_bbox_expand_ratio, support_minimum_span_m)
''');g.write_text(s)
s=r.read_text();needle='from alicia_flexible_grasp.grasp.contact_evidence import ('
s=s.replace(needle,'''from alicia_flexible_grasp.grasp.contact_height_margin import evaluate_contact_height_margin
from alicia_flexible_grasp.grasp.contact_height_placement import place_contact_height
'''+needle,1)
needle='            support_bbox_expand_ratio=float(\n                getattr(self, \'geometry_support_bbox_expand_ratio\', 0.30)\n            ),'
assert s.count(needle)==1
s=s.replace(needle,needle+'''
            support_minimum_span_m=(max(
                float(self.gripper_geometry.finger_size_xyz_m[0]),
                float(self.gripper_geometry.max_inner_gap_m)
                + 2. * float(self.gripper_geometry.finger_size_xyz_m[1]))
                + 2. * float(self.gripper_geometry.support_clearance_m)),''')
needle='    def _reproject_tabletop_stable_pose_for_support_clearance(\n'
helper='''    def _contact_height_evidence_limit(self):
        cfg = getattr(self, 'grasp_config', {})
        enabled = cfg.get('contact_height_registration_margin_enabled', False)
        if type(enabled) is not bool:
            raise CandidateContractError('CONTACT_HEIGHT_REGISTRATION_MARGIN',
                                         'height margin flag must be boolean')
        if not enabled:
            return None
        limit = cfg.get('handeye_translation_max_error_m')
        if (isinstance(limit, (bool, np.bool_))
                or not isinstance(limit, (int, float, np.number))
                or not math.isfinite(float(limit)) or float(limit) <= 0.):
            raise CandidateContractError('CONTACT_HEIGHT_REGISTRATION_MARGIN',
                                         'height evidence limit must be finite and positive')
        return float(limit)

    def _contact_height_geometry(self, transform, geometry, center, bounds, axis, minimum):
        return dict(gripper=self.gripper_geometry,
            actual_T_base_tool0=transform, planned_T_base_tool0=transform,
            contact_center_base=center, contact_height_axis_base=axis,
            contact_height_bounds_m=bounds, minimum_contact_overlap_m=minimum,
            support_normal_base=geometry.support_normal_base,
            support_offset_m=geometry.support_offset_m,
            opening_width_m=min(float(self.gripper_geometry.max_inner_gap_m),
                                float(self.gripper_physical_open_width_m)))

'''
assert needle in s;s=s.replace(needle,helper+needle,1)
needle='''                            else:
                                target_points_base = surface.points_base
                        explicit_gate_kwargs = {'''
replacement='''                            else:
                                target_points_base = surface.points_base
                                if str(stable.model_choice) == 'unknown_tabletop':
                                    limit = self._contact_height_evidence_limit()
                                    if limit is not None:
                                        transform, height_placement = place_contact_height(
                                            translation_evidence_limit_m=limit,
                                            **self._contact_height_geometry(transform,
                                                geometry, stable.center_base_xyz,
                                                measured_bounds, measured_axis,
                                                self._phase_contact_overlap_requirement_m(
                                                    True, stage_profile)))
                                        if transform is None:
                                            raise CandidateContractError(
                                                'CONTACT_HEIGHT_REGISTRATION_MARGIN',
                                                height_placement['reason'])
                                        grasp_pose = make_pose_stamped('base_link',
                                            transform[:3, 3], quaternion_from_matrix(transform),
                                            stamp=grasp_pose.header.stamp)
                                        sequence, stage_profile = self._make_contact_sequence(
                                            grasp_pose, geometry,
                                            insertion_axis_base=transform[:3, 2],
                                            snapshot=prepared.snapshot)
                        explicit_gate_kwargs = {'''
assert s.count(needle)==1;s=s.replace(needle,replacement)
needle="        runtime['registration_support_lift_m'] = support_lift_m\n"
replacement=needle+'''        if (str(plan.model_choice) == 'unknown_tabletop'
                and str(plan.diagnostic) == CONTACT_EXECUTION_PLAN):
            limit = self._contact_height_evidence_limit()
            if limit is not None:
                evidence = decode_contact_evidence(plan, required=True)
                transform = pose_matrix(self._rich_plan_execution_sequence(plan).grasp)
                report = evaluate_contact_height_margin(
                    translation_evidence_limit_m=limit,
                    **self._contact_height_geometry(transform, geometry,
                        evidence['contact_center_base'], evidence['contact_height_bounds_m'],
                        evidence['contact_height_axis_base'], evidence['minimum_contact_overlap_m']))
                runtime['final_contact_height_margin'] = report
                if report['ok'] is not True:
                    raise CandidateContractError('CONTACT_HEIGHT_REGISTRATION_MARGIN', report['reason'])
'''
assert s.count(needle)==1;s=s.replace(needle,replacement)
# Include final evidence even for rejected candidates in the persisted lineage.
needle="                'final_contact_rejection': deepcopy(runtime.get('final_contact_rejection')),"
if needle in s:s=s.replace(needle,needle+"\n                'final_contact_height_margin': deepcopy(runtime.get('final_contact_height_margin')),")
r.write_text(s)
