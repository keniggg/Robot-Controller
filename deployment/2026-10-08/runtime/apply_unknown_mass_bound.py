from pathlib import Path
import json,shutil
root=Path.cwd();out=Path(json.load(open('.runtime/current_resumed_grasp_session.json'))['directory'])
p=root/'src/alicia_flexible_grasp_supervisor/src/alicia_flexible_grasp/vision/mujoco_digital_twin_client.py'
shutil.copyfile(p,out/'mujoco_client_before_declared_mass_bound.py');s=p.read_text()
needle='\ndef build_mujoco_payload(plan, joint_names, joint_positions, gripper_config=None):'
helper='''
def _apply_unknown_mass_bound(plan, policy, mass, assumptions):
    """Apply the operator's persistent operating-domain bound to unknowns.

    This is an explicit declared upper limit, not a weight estimate or a
    measurement. The payload always tests that upper limit; dynamics and
    physical acceptance gates otherwise remain unchanged.
    """
    if policy is None or plan.model_choice != 'unknown_tabletop':
        return mass, assumptions
    if (not isinstance(policy, dict)
            or policy.get('source') != 'operator_declared_operational_limit'
            or policy.get('scope') != 'all_unknown_objects'
            or not isinstance(policy.get('operator_statement'), str)
            or not policy['operator_statement'].strip()):
        raise ValueError('unknown mass bound requires explicit operator provenance')
    raw = policy.get('maximum_mass_kg')
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise ValueError('unknown maximum mass must be a finite number')
    maximum = _finite_float(raw, 'unknown maximum_mass_kg')
    if not 0.0 < maximum <= assumptions['operational_mass_ceiling_kg']:
        raise ValueError('unknown maximum mass exceeds the operating range')
    evidence = dict(policy)
    updated = dict(assumptions,
        source='operator_declared_unknown_mass_upper_bound',
        previous_envelope_mass_kg=mass,
        declared_upper_bound_kg=maximum,
        simulation_mass_kg=maximum,
        measured_upper_bound=False,
        operator_mass_policy=evidence)
    return maximum, updated

'''
assert s.count(needle)==1;s=s.replace(needle,'\n'+helper+needle)
needle="    mass, dynamics_assumptions = _apply_target_mass_estimate(\n        plan, object_config, root_config.get('target_mass_evidence'), mass, dynamics_assumptions)"
replacement=needle+"\n    mass, dynamics_assumptions = _apply_unknown_mass_bound(\n        plan, root_config.get('unknown_object_mass_bound'), mass, dynamics_assumptions)"
assert s.count(needle)==1;s=s.replace(needle,replacement);p.write_text(s)
p=root/'src/alicia_flexible_grasp_supervisor/config/grasp_params.yaml';shutil.copyfile(p,out/'grasp_params_before_declared_mass_bound.yaml');s=p.read_text();needle='mujoco_digital_twin:\n  enabled: true\n';replacement=needle+'''  # User-declared operating domain: all future unknown objects are <= 20 g.
  # Applies only to unknown_tabletop. This is not a measured per-object mass.
  unknown_object_mass_bound:
    source: operator_declared_operational_limit
    scope: all_unknown_objects
    maximum_mass_kg: 0.020
    operator_statement: "以后所有的未知物体不会超过20g"
''';assert s.count(needle)==1;p.write_text(s.replace(needle,replacement));print('Added unknown-only declared upper-bound policy.')
