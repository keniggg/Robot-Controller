#!/usr/bin/env python3
"""Offline frozen-request solver diagnostics; no ROS or HTTP calls."""
import argparse
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import types

W = Path(__file__).resolve().parents[1]
EXTERNAL = Path('/media/zhuyupei/SanStand/实验记录/Robot-Controller')
SERVICE = EXTERNAL / 'ros_log/candidate_simulation_full_start_20261006T024226Z_aczfr1vf/service'
OUT = EXTERNAL / 'ros_log/grasp_explicit_request_20261006T045404Z/simulation_symmetry_diagnostic'
INPUTS = {
    'short_85_0': '1791262459112961725_2490b115b3d84f1ab3375254d385ec96',
    'flip_85_1': '1791262469105627511_ce33bd76f0634b0e94650373052f5760',
}

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def dump(p, value):
    p.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--variant', default='baseline', choices=['baseline', 'baseline_detailed', 'baseline_reference', 'velocity_ff', 'dt1ms', 'dt05ms', 'iter200', 'dt05ms_iter200', 'lift_dt1ms', 'lift_dt05ms'])
    parser.add_argument('--branch', default='both', choices=['both', *INPUTS])
    parser.add_argument('--repaired-source', action='store_true')
    args = parser.parse_args()
    if not os.path.ismount('/media/zhuyupei/SanStand'):
        raise RuntimeError('SanStand unavailable; no fallback')
    OUT.mkdir(exist_ok=True)
    wrapper_path = W / '.runtime/candidate_simulation/run_candidate.py'
    spec = importlib.util.spec_from_file_location('offline_candidate_wrapper', wrapper_path)
    wrapper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(wrapper)
    if args.repaired_source:
        pins = json.loads((W/'.runtime/candidate_simulation/pins.json').read_text())
        for filename,digest in pins['files'].items():
            if filename == str(W/'tools/mujoco_digital_twin_server.py'):
                digest = '5342482a2ccdff1e555bb914f6a049727892aec7a769f5c907faaff59621fb59'
            assert sha(filename)==digest, filename
    else:
        pins = wrapper.verify_pins()
    engine = wrapper.load_engine(pins)
    loaded_xml = SERVICE / 'loaded_candidate.xml'
    assert sha(loaded_xml) == pins['model_xml_sha256']
    for branch in INPUTS if args.branch == 'both' else [args.branch]:
        run = OUT / (('repaired_' if args.repaired_source else '') + args.variant + '_' + branch)
        run.mkdir(exist_ok=False)
        origin = SERVICE / 'requests' / INPUTS[branch]
        raw = (origin / 'request.raw.json').read_bytes()
        payload = json.loads(raw)
        (run / 'request.raw.json').write_bytes(raw)
        archived = json.loads((origin / 'response.json').read_text())
        if args.repaired_source:
            backend=engine.MujocoDigitalTwinBackend(model_xml=loaded_xml,
                max_snapshot_age_sec=120.,ros_sync_joint_states=False)
            backend._import_mujoco()
            model,_,_=backend._model_for_payload(dict(object_model=dict(type='box',
                size_xyz_m=[.020,.014,.030],mass_kg=.020,friction=[1.,.1,.1])))
            compiled=dict(compiled=True,original_gripper_contract_passed=True,
                nq=model.nq,nv=model.nv,nu=model.nu,health=backend.health(),
                constructor_semantics='exact original wrapper constructor without wrapper source-pin health guard',
                old_pins_preserved=True,updated_source_sha256=sha(W/'tools/mujoco_digital_twin_server.py'))
        else:
            backend, compiled = wrapper.build_backend(engine, pins, loaded_xml)
        # Validate schema and freshness at the captured request time. This is
        # replay-only; do not relabel stale evidence as live or send a request.
        captured_ns = json.loads((origin / 'request_metadata.json').read_text())['received_ns']
        checked = engine._validate_v3_payload(copy.deepcopy(payload), now_sec=captured_ns * 1e-9,
                                              max_snapshot_age_sec=120.0)
        original_model = backend._model_for_payload
        options = {}
        def model_for_payload(self, p):
            model, data, meta = original_model(p)
            before = {k: float(getattr(model.opt, k)) for k in ['timestep', 'tolerance', 'iterations', 'ls_iterations', 'solver', 'integrator', 'cone']}
            if args.variant in ('dt1ms', 'dt05ms', 'dt05ms_iter200'):
                model.opt.timestep = .001 if args.variant == 'dt1ms' else .0005
            if args.variant in ('iter200', 'dt05ms_iter200'):
                model.opt.iterations = 200
            after = {k: float(getattr(model.opt, k)) for k in before}
            options.update(before=before, after=after)
            return model, data, meta
        backend._model_for_payload = types.MethodType(model_for_payload, backend)
        reference = {}
        if args.variant in ('baseline_reference', 'velocity_ff'):
            original_apply = backend._apply_velocity_consistent_arm_reference
            def apply(self, model, data, joints, actuators, start, target, dt):
                reference.clear()
                reference.update(start=list(start), target=list(target), dt=dt,
                    entry_qpos=[float(data.qpos[model.jnt_qposadr[j]]) for j in joints],
                    entry_qvel=[float(data.qvel[model.jnt_dofadr[j]]) for j in joints])
                if args.variant == 'velocity_ff':
                    result=0.
                    for j,a,s,t in zip(joints,actuators,start,target):
                        speed=(float(t)-float(s))/dt
                        data.qpos[model.jnt_qposadr[j]]=float(s)
                        data.qvel[model.jnt_dofadr[j]]=speed
                        kp=float(model.actuator_gainprm[a,0]);kv=-float(model.actuator_biasprm[a,2])
                        if kp <= 0 or kv < 0: raise RuntimeError('unexpected position actuator')
                        data.ctrl[a]=float(t)+kv/kp*speed
                        result=max(result,abs(speed))
                    self._mujoco.mj_forward(model,data)
                else:
                    result = original_apply(model, data, joints, actuators, start, target, dt)
                reference['qacc_after_forward']=[float(data.qacc[model.jnt_dofadr[j]]) for j in joints]
                return result
            backend._apply_velocity_consistent_arm_reference = types.MethodType(apply, backend)
        if args.variant in ('lift_dt1ms', 'lift_dt05ms'):
            original_lift = backend._simulate_lift
            def lift(self, *a, **kw):
                model = kw['model'] if 'model' in kw else a[0]
                retained = kw['retained_data'] if 'retained_data' in kw else a[1]
                options['lift_entry_retained_state'] = dict(time_sec=float(retained.time),
                    qpos=retained.qpos.tolist(), qvel=retained.qvel.tolist())
                model.opt.timestep = .001 if args.variant == 'lift_dt1ms' else .0005
                options['lift_timestep_sec'] = float(model.opt.timestep)
                return original_lift(*a, **kw)
            backend._simulate_lift = types.MethodType(lift, backend)
        telemetry = []
        original_classify = backend._classify_current_contacts
        def classify(self, model, data, meta):
            verdict = original_classify(model, data, meta)
            pairs = self._body_contacts(model, data)
            support_contacts = []
            for c in pairs:
                a, b, distance = engine._contact_parts(c)
                if set((a, b)) == {'detected_support', 'target_object'}:
                    support_contacts.append(distance)
            row = dict(time_sec=float(data.time), two_sided=verdict.two_sided,
                disallowed=verdict.disallowed_collision,
                reasons=list(verdict.disallowed_contacts),
                object_center_m=data.xpos[meta['object_body']].tolist(),
                support_contact_distances_m=support_contacts)
            if args.variant in ('baseline_detailed', 'baseline_reference', 'velocity_ff'):
                row['dynamic_gripper'] = self._gripper_dynamic_state(model, data, meta)
                row['actual_contacts'] = []
                for c in data.contact[:data.ncon]:
                    names = [self._geom_body_name(model, c.geom1), self._geom_body_name(model, c.geom2)]
                    if 'target_object' in names:
                        row['actual_contacts'].append(dict(bodies=names, distance_m=float(c.dist),
                            friction=c.friction.tolist(), dim=int(c.dim), solref=c.solref.tolist(), solimp=c.solimp.tolist()))
            if args.variant in ('baseline_reference', 'velocity_ff'):
                row['reference']=copy.deepcopy(reference)
                row['arm_qpos']=[float(data.qpos[model.jnt_qposadr[j]]) for j in meta['arm_joints']]
                row['arm_qvel']=[float(data.qvel[model.jnt_dofadr[j]]) for j in meta['arm_joints']]
                row['arm_qacc']=[float(data.qacc[model.jnt_dofadr[j]]) for j in meta['arm_joints']]
                jadr=model.jnt_dofadr[meta['object_joint']]
                row['object_qvel']=data.qvel[jadr:jadr+6].tolist()
            telemetry.append(row)
            return verdict
        backend._classify_current_contacts = types.MethodType(classify, backend)
        metadata = dict(schema=1, offline_only=True, robot_commands_sent=False,
            network_calls=False, source_request=str(origin/'request.raw.json'),
            request_sha256=hashlib.sha256(raw).hexdigest(), replay_variant=args.variant,
            source_sha256=sha(W/'tools/mujoco_digital_twin_server.py'),
            wrapper_sha256=sha(wrapper_path), script_sha256=sha(__file__),
            loaded_model_xml=str(loaded_xml), loaded_model_xml_sha256=sha(loaded_xml),
            compiled=compiled, pins=pins,
            unchanged_contact_penetration_tolerance_m=engine.OBJECT_SUPPORT_PENETRATION_TOLERANCE_M,
            original_source_timestamps_preserved=True,
            protocol_validation_now_sec=captured_ns*1e-9,
            conclusion_scope='offline numerical convergence only; no physical reachability or grasp proof')
        dump(run/'metadata.json', metadata)
        started=time.monotonic()
        result = backend._simulate_validated_grasp(checked, payload['plan_id'])
        wall=time.monotonic()-started
        dump(run/'response.json',result)
        dump(run/'contact_samples.json',telemetry)
        differences={key: {'archived': value, 'replayed': result.get(key)}
                     for key,value in archived.items() if key not in ('ok',) and result.get(key)!=value}
        all_support=[v for row in telemetry for v in row['support_contact_distances_m']]
        summary = dict(branch=branch, variant=args.variant, wall_sec=wall, options=options,
            failure_code=result.get('failure_code'),failure_reason=result.get('failure_reason'),
            collision_free=result.get('collision_free'), contact_success=result.get('contact_success'),
            lift_success=result.get('lift_success'), lift_evidence=result.get('lift_evidence'),
            min_observed_support_contact_m=min(all_support) if all_support else None,
            classified_sample_count=len(telemetry), archived_response_differences=differences,
            request_unchanged=(json.loads(raw)==payload), physical_validation=False)
        dump(run/'summary.json',summary)
        assert sha(loaded_xml)==pins['model_xml_sha256']
        assert sha(W/'tools/mujoco_digital_twin_server.py')==(metadata['source_sha256'])
        print(json.dumps(dict(branch=branch, variant=args.variant, wall_sec=wall,
            failure_code=result.get('failure_code'),failure_reason=result.get('failure_reason'),
            min_support=summary['min_observed_support_contact_m'],
            differences=list(differences), output=str(run)),ensure_ascii=False),flush=True)

if __name__ == '__main__':
    main()
