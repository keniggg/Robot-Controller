#!/usr/bin/env python3
"""Combined WSL GraspNet + MuJoCo digital twin server.

This server is intended to run in the Windows/WSL2 GPU environment.  It keeps
the existing GraspNet baseline /predict protocol and adds a MuJoCo
/simulate_grasp pre-execution gate for the Alicia-D grasp sequence.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np


TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from graspnet_baseline_server import (  # noqa: E402
    GraspNetBaselineBackend,
    MockGraspNetBackend as _BaselineMockGraspNetBackend,
)


DEFAULT_MODEL_XML = (
    REPO_ROOT
    / 'src'
    / 'arm-mujoco'
    / 'synriard'
    / 'mjcf'
    / 'Alicia_D_v5_6'
    / 'Alicia_D_v5_6_gripper_50mm.xml'
)


def make_server(host, port, grasp_backend, sim_backend):
    server = ThreadingHTTPServer((host, int(port)), MujocoDigitalTwinHTTPHandler)
    server.grasp_backend = grasp_backend
    server.sim_backend = sim_backend
    return server


class MujocoDigitalTwinHTTPHandler(BaseHTTPRequestHandler):
    server_version = 'AliciaMujocoDigitalTwinHTTP/1.0'

    def do_GET(self):
        if self.path != '/health':
            self._send_json(404, {'ok': False, 'error': 'unknown path'})
            return
        grasp_health = self.server.grasp_backend.health()
        sim_health = self.server.sim_backend.health()
        self._send_json(
            200,
            {
                'ok': bool(grasp_health.get('ok', False)) and bool(sim_health.get('ok', False)),
                'backend': grasp_health.get('backend', 'unknown'),
                'loaded': grasp_health.get('loaded', False),
                'grasp_backend': grasp_health,
                'digital_twin': sim_health,
            },
        )

    def do_POST(self):
        handlers = {
            '/predict': self._handle_predict,
            '/sync_joint_state': self._handle_sync_joint_state,
            '/simulate_grasp': self._handle_simulate_grasp,
        }
        handler = handlers.get(self.path)
        if handler is None:
            self._send_json(404, {'ok': False, 'error': 'unknown path'})
            return
        try:
            payload = self._read_payload()
            self._send_json(200, handler(payload))
        except Exception as exc:
            self._send_json(200, {'ok': False, 'error': str(exc)})

    def _handle_predict(self, payload):
        candidates = self.server.grasp_backend.predict(payload)
        return {
            'ok': True,
            'backend': self.server.grasp_backend.name,
            'candidates': candidates,
        }

    def _handle_sync_joint_state(self, payload):
        self.server.sim_backend.update_joint_state(payload)
        return {'ok': True, 'backend': self.server.sim_backend.name, 'message': 'joint state synced'}

    def _handle_simulate_grasp(self, payload):
        result = self.server.sim_backend.simulate_grasp(payload)
        result['ok'] = True
        return result

    def _read_payload(self):
        length = int(self.headers.get('Content-Length', '0'))
        return json.loads(self.rfile.read(length).decode('utf-8'))

    def _send_json(self, status, payload):
        data = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        sys.stderr.write('[%s] %s\n' % (self.log_date_time_string(), fmt % args))


class MockGraspNetBackend(_BaselineMockGraspNetBackend):
    def predict(self, payload):
        if payload.get('encoding') == 'mock':
            return [
                {
                    'score': 1.0,
                    'width_m': 0.05,
                    'translation_m': [0.30, 0.0, 0.20],
                    'rotation_matrix': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                }
            ]
        return super().predict(payload)


class JointStateCache:
    def __init__(self):
        self._lock = threading.Lock()
        self.names = []
        self.positions = []
        self.stamp = 0.0
        self.source = 'none'

    def update(self, names, positions, source='http'):
        with self._lock:
            self.names = [str(name) for name in names]
            self.positions = [float(value) for value in positions]
            self.stamp = time.time()
            self.source = str(source)

    def snapshot(self):
        with self._lock:
            return list(self.names), list(self.positions), float(self.stamp), str(self.source)

    def age_sec(self):
        with self._lock:
            if not self.stamp:
                return float('inf')
            return max(0.0, time.time() - float(self.stamp))


class MockDigitalTwinBackend:
    name = 'mock_mujoco'

    def __init__(self):
        self.joint_cache = JointStateCache()

    def health(self):
        return {
            'ok': True,
            'backend': self.name,
            'joint_state_age_sec': self.joint_cache.age_sec(),
        }

    def update_joint_state(self, payload):
        self.joint_cache.update(payload.get('joint_names') or [], payload.get('joint_positions') or [], source='http')

    def simulate_grasp(self, payload):
        sequence = payload.get('grasp_sequence_base') or []
        ok = len(sequence) >= 4
        return {
            'backend': self.name,
            'simulation_ok': bool(ok),
            'score': 90 if ok else 0,
            'ik_success': bool(ok),
            'collision_free': bool(ok),
            'contact_success': bool(ok),
            'lift_success': bool(ok),
            'failure_reason': '' if ok else 'grasp_sequence_base must contain pregrasp/approach/grasp/lift',
            'diagnosis': ['mock digital twin accepted 4-stage grasp sequence'] if ok else [],
            'used_joint_state_source': self.joint_cache.snapshot()[3],
        }


class MujocoDigitalTwinBackend:
    name = 'mujoco'

    def __init__(
        self,
        model_xml=DEFAULT_MODEL_XML,
        pass_score=80,
        max_joint_state_age_sec=2.0,
        ros_sync_joint_states=False,
        ros_joint_state_topic='/joint_states',
        ee_orientation_body='Link6',
        left_finger_body='Link7',
        right_finger_body='Link8',
    ):
        self.model_xml = Path(model_xml).expanduser()
        self.pass_score = int(pass_score)
        self.max_joint_state_age_sec = float(max_joint_state_age_sec)
        self.ros_sync_joint_states = bool(ros_sync_joint_states)
        self.ros_joint_state_topic = str(ros_joint_state_topic)
        self.ee_orientation_body = str(ee_orientation_body)
        self.left_finger_body = str(left_finger_body)
        self.right_finger_body = str(right_finger_body)
        self.joint_cache = JointStateCache()
        self._lock = threading.Lock()
        self._model_cache = {}
        self._mujoco = None
        if self.ros_sync_joint_states:
            self._start_ros_joint_state_subscriber()

    def health(self):
        missing = []
        if not self.model_xml.exists():
            missing.append('model_xml not found: %s' % self.model_xml)
        try:
            self._import_mujoco()
            mujoco_version = getattr(self._mujoco, '__version__', 'unknown')
        except Exception as exc:
            missing.append('mujoco import failed: %s' % exc)
            mujoco_version = 'missing'
        return {
            'ok': not missing,
            'backend': self.name,
            'model_xml': str(self.model_xml),
            'mujoco': str(mujoco_version),
            'joint_state_age_sec': self.joint_cache.age_sec(),
            'ros_sync_joint_states': self.ros_sync_joint_states,
            'missing': missing,
        }

    def update_joint_state(self, payload):
        self.joint_cache.update(payload.get('joint_names') or [], payload.get('joint_positions') or [], source='http')

    def simulate_grasp(self, payload):
        with self._lock:
            self._import_mujoco()
            model, data, meta = self._model_for_payload(payload)
            self._apply_joint_state(model, data, payload)
            self._apply_gripper_width(model, data, float(payload.get('gripper_width_m', 0.05)))
            self._apply_object_pose(model, data, payload)
            self._mujoco.mj_forward(model, data)
            sequence = _parse_grasp_sequence(payload)
            if len(sequence) < 4:
                return self._failure('grasp_sequence_base must contain pregrasp/approach/grasp/lift')

            ik_results = []
            trajectory = []
            current_data = self._copy_data(model, data)
            for target in sequence[:4]:
                result = self._solve_ik(model, current_data, target['position'], target['rotation_matrix'], meta)
                ik_results.append((target['name'], result))
                if not result['success']:
                    return self._score_response(
                        ik_results,
                        collision_free=False,
                        contact_success=False,
                        lift_success=False,
                        failure_reason='IK failed at %s: position error %.4fm orientation error %.4f'
                        % (target['name'], result['position_error_m'], result['orientation_error']),
                        diagnosis=['IK failed for %s' % target['name']],
                    )
                self._set_arm_qpos(model, current_data, meta['arm_joints'], result['joint_positions'])
                trajectory.append((target['name'], result['joint_positions']))

            collision_free, collision_diag = self._check_trajectory_collisions(model, data, meta, trajectory)
            contact_success, contact_diag = self._simulate_close_contact(model, data, meta, trajectory[2][1], payload)
            lift_success, lift_diag = self._simulate_lift(model, data, meta, trajectory[2][1], trajectory[3][1], payload)
            diagnosis = []
            diagnosis.extend(collision_diag)
            diagnosis.extend(contact_diag)
            diagnosis.extend(lift_diag)
            failure_reason = ''
            if not collision_free:
                failure_reason = collision_diag[0] if collision_diag else 'trajectory collision detected'
            elif not contact_success:
                failure_reason = contact_diag[0] if contact_diag else 'gripper did not form two-sided object contact'
            elif not lift_success:
                failure_reason = lift_diag[0] if lift_diag else 'object did not lift with gripper'
            return self._score_response(
                ik_results,
                collision_free=collision_free,
                contact_success=contact_success,
                lift_success=lift_success,
                failure_reason=failure_reason,
                diagnosis=diagnosis,
            )

    def _failure(self, reason):
        return {
            'backend': self.name,
            'simulation_ok': False,
            'score': 0,
            'ik_success': False,
            'collision_free': False,
            'contact_success': False,
            'lift_success': False,
            'failure_reason': str(reason),
            'diagnosis': [str(reason)],
            'used_joint_state_source': self.joint_cache.snapshot()[3],
        }

    def _score_response(self, ik_results, collision_free, contact_success, lift_success, failure_reason, diagnosis):
        ik_success = bool(ik_results) and all(item[1]['success'] for item in ik_results)
        orientation_ok = bool(ik_results) and all(item[1]['orientation_error'] <= 0.18 for item in ik_results)
        score = 0
        score += 20 if ik_success else 0
        score += 20 if collision_free else 0
        score += 15 if orientation_ok else 0
        score += 20 if contact_success else 0
        score += 20 if lift_success else 0
        score += 5 if not failure_reason else 0
        return {
            'backend': self.name,
            'simulation_ok': bool(score >= self.pass_score and ik_success and collision_free),
            'score': int(score),
            'ik_success': bool(ik_success),
            'collision_free': bool(collision_free),
            'contact_success': bool(contact_success),
            'lift_success': bool(lift_success),
            'failure_reason': str(failure_reason or ''),
            'diagnosis': diagnosis or ['simulation score=%d' % score],
            'ik_results': [
                {
                    'name': name,
                    'success': bool(result['success']),
                    'position_error_m': float(result['position_error_m']),
                    'orientation_error': float(result['orientation_error']),
                    'iterations': int(result['iterations']),
                }
                for name, result in ik_results
            ],
            'used_joint_state_source': self.joint_cache.snapshot()[3],
        }

    def _import_mujoco(self):
        if self._mujoco is None:
            import mujoco

            self._mujoco = mujoco
        return self._mujoco

    def _start_ros_joint_state_subscriber(self):
        try:
            import rospy
            from sensor_msgs.msg import JointState
        except Exception as exc:
            sys.stderr.write('WARNING: ROS joint state sync disabled; rospy import failed: %s\n' % exc)
            self.ros_sync_joint_states = False
            return
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('mujoco_digital_twin_joint_sync', anonymous=True, disable_signals=True)
            rospy.Subscriber(self.ros_joint_state_topic, JointState, self._ros_joint_state_cb, queue_size=1)
        except Exception as exc:
            sys.stderr.write('WARNING: ROS joint state sync disabled; subscriber failed: %s\n' % exc)
            self.ros_sync_joint_states = False

    def _ros_joint_state_cb(self, msg):
        self.joint_cache.update(getattr(msg, 'name', []), getattr(msg, 'position', []), source='ros')

    def _model_for_payload(self, payload):
        object_model = dict(payload.get('object_model') or {})
        key = json.dumps(object_model, sort_keys=True)
        cached = self._model_cache.get(key)
        if cached is not None:
            model = cached[0]
            return model, self._mujoco.MjData(model), cached[1]
        xml = self.model_xml.read_text(encoding='utf-8')
        xml = _inject_options(xml)
        xml = _inject_actuators(xml)
        xml = _inject_target_object(xml, object_model)
        cwd = os.getcwd()
        try:
            os.chdir(str(self.model_xml.parent))
            model = self._mujoco.MjModel.from_xml_string(xml)
        finally:
            os.chdir(cwd)
        meta = self._model_meta(model)
        self._model_cache[key] = (model, meta)
        return model, self._mujoco.MjData(model), meta

    def _model_meta(self, model):
        return {
            'arm_joints': [self._joint_id(model, name) for name in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']],
            'left_finger_joint': self._joint_id(model, 'left_finger'),
            'right_finger_joint': self._joint_id(model, 'right_finger'),
            'left_body': self._body_id(model, self.left_finger_body),
            'right_body': self._body_id(model, self.right_finger_body),
            'orientation_body': self._body_id(model, self.ee_orientation_body),
            'object_body': self._body_id(model, 'target_object'),
            'object_joint': self._joint_id(model, 'target_object_joint', required=False),
        }

    def _joint_id(self, model, name, required=True):
        joint_id = self._mujoco.mj_name2id(model, self._mujoco.mjtObj.mjOBJ_JOINT, str(name))
        if required and joint_id < 0:
            raise RuntimeError('MuJoCo joint not found: %s' % name)
        return joint_id

    def _body_id(self, model, name):
        body_id = self._mujoco.mj_name2id(model, self._mujoco.mjtObj.mjOBJ_BODY, str(name))
        if body_id < 0:
            raise RuntimeError('MuJoCo body not found: %s' % name)
        return body_id

    def _apply_joint_state(self, model, data, payload):
        names = payload.get('joint_names')
        positions = payload.get('joint_positions')
        source = 'request'
        if not names or not positions:
            names, positions, stamp, source = self.joint_cache.snapshot()
            if not names or not positions:
                raise RuntimeError('no joint state supplied and WSL ROS/http joint cache is empty')
            if self.max_joint_state_age_sec > 0.0 and time.time() - stamp > self.max_joint_state_age_sec:
                raise RuntimeError('cached joint state is stale %.2fs' % (time.time() - stamp))
        mapping = {str(name): float(value) for name, value in zip(names, positions)}
        for joint_name in ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']:
            if joint_name not in mapping:
                continue
            joint_id = self._joint_id(model, joint_name)
            data.qpos[model.jnt_qposadr[joint_id]] = mapping[joint_name]
        self.joint_cache.update(names, positions, source=source)

    def _apply_gripper_width(self, model, data, width_m):
        width = max(0.0, min(0.05, float(width_m)))
        left = -0.5 * width
        right = 0.5 * width
        left_id = self._joint_id(model, 'left_finger')
        right_id = self._joint_id(model, 'right_finger')
        data.qpos[model.jnt_qposadr[left_id]] = left
        data.qpos[model.jnt_qposadr[right_id]] = right

    def _apply_object_pose(self, model, data, payload):
        obj = dict(payload.get('object_pose_base') or {})
        joint_id = self._joint_id(model, 'target_object_joint', required=False)
        if joint_id < 0:
            return
        pos = _vector3(obj.get('position'), [0.30, 0.0, 0.04])
        quat_xyzw = _normalize_quat(obj.get('quaternion_xyzw') or [0.0, 0.0, 0.0, 1.0])
        qadr = model.jnt_qposadr[joint_id]
        data.qpos[qadr : qadr + 3] = pos
        data.qpos[qadr + 3 : qadr + 7] = [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]]

    def _copy_data(self, model, data):
        copied = self._mujoco.MjData(model)
        copied.qpos[:] = data.qpos[:]
        copied.qvel[:] = 0.0
        self._mujoco.mj_forward(model, copied)
        return copied

    def _solve_ik(self, model, seed_data, target_pos, target_rot, meta):
        data = self._copy_data(model, seed_data)
        arm_joints = meta['arm_joints']
        dofs = [model.jnt_dofadr[joint] for joint in arm_joints]
        lower = np.asarray([model.jnt_range[joint][0] for joint in arm_joints], dtype=float)
        upper = np.asarray([model.jnt_range[joint][1] for joint in arm_joints], dtype=float)
        target_pos = np.asarray(target_pos, dtype=float).reshape(3)
        target_rot = np.asarray(target_rot, dtype=float).reshape(3, 3)
        jacp_l = np.zeros((3, model.nv))
        jacr_l = np.zeros((3, model.nv))
        jacp_r = np.zeros((3, model.nv))
        jacr_r = np.zeros((3, model.nv))
        jacp_o = np.zeros((3, model.nv))
        jacr_o = np.zeros((3, model.nv))
        lam = 0.025
        final_pos_err = float('inf')
        final_rot_err = float('inf')
        iterations = 0
        for iterations in range(1, 241):
            self._mujoco.mj_forward(model, data)
            center = self._gripper_center(data, meta)
            pos_err = target_pos - center
            rot_err = _orientation_error(data.xmat[meta['orientation_body']].reshape(3, 3), target_rot)
            final_pos_err = float(np.linalg.norm(pos_err))
            final_rot_err = float(np.linalg.norm(rot_err))
            if final_pos_err <= 0.005 and final_rot_err <= 0.16:
                break
            self._mujoco.mj_jacBody(model, data, jacp_l, jacr_l, meta['left_body'])
            self._mujoco.mj_jacBody(model, data, jacp_r, jacr_r, meta['right_body'])
            self._mujoco.mj_jacBody(model, data, jacp_o, jacr_o, meta['orientation_body'])
            jac_pos = ((jacp_l + jacp_r) * 0.5)[:, dofs]
            jac = np.vstack((jac_pos, 0.25 * jacr_o[:, dofs]))
            err = np.concatenate((pos_err, 0.25 * rot_err))
            lhs = jac.T @ jac + lam * np.eye(len(dofs))
            rhs = jac.T @ err
            try:
                dq = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                dq = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            dq = np.clip(dq, -0.08, 0.08)
            q = np.asarray([data.qpos[model.jnt_qposadr[joint]] for joint in arm_joints], dtype=float)
            q = np.clip(q + 0.45 * dq, lower, upper)
            self._set_arm_qpos(model, data, arm_joints, q)
        success = bool(final_pos_err <= 0.005 and final_rot_err <= 0.16)
        q = np.asarray([data.qpos[model.jnt_qposadr[joint]] for joint in arm_joints], dtype=float)
        return {
            'success': success,
            'joint_positions': q.tolist(),
            'position_error_m': final_pos_err,
            'orientation_error': final_rot_err,
            'iterations': iterations,
        }

    def _set_arm_qpos(self, model, data, joints, values):
        for joint_id, value in zip(joints, values):
            data.qpos[model.jnt_qposadr[joint_id]] = float(value)
        self._mujoco.mj_forward(model, data)

    def _gripper_center(self, data, meta):
        return (data.xpos[meta['left_body']] + data.xpos[meta['right_body']]) * 0.5

    def _check_trajectory_collisions(self, model, seed_data, meta, trajectory):
        data = self._copy_data(model, seed_data)
        previous = np.asarray([data.qpos[model.jnt_qposadr[j]] for j in meta['arm_joints']], dtype=float)
        diagnosis = []
        for name, target in trajectory:
            target = np.asarray(target, dtype=float)
            for alpha in np.linspace(0.0, 1.0, 20):
                q = (1.0 - alpha) * previous + alpha * target
                self._set_arm_qpos(model, data, meta['arm_joints'], q)
                bad = self._bad_contacts(model, data, allow_finger_object=(name in ('grasp', 'lift')))
                if bad:
                    diagnosis.append('collision at %s: %s' % (name, bad[0]))
                    return False, diagnosis
            previous = target
        diagnosis.append('trajectory collision check passed')
        return True, diagnosis

    def _simulate_close_contact(self, model, seed_data, meta, grasp_q, payload):
        data = self._copy_data(model, seed_data)
        self._set_arm_qpos(model, data, meta['arm_joints'], grasp_q)
        open_width = max(0.0, min(0.05, float(payload.get('gripper_width_m', 0.05))))
        close_width = max(0.0, min(open_width, float(payload.get('close_width_m', 0.0))))
        for width in np.linspace(open_width, close_width, 35):
            self._apply_gripper_width(model, data, width)
            self._mujoco.mj_step(model, data)
        left, right = self._finger_object_contacts(model, data, meta)
        if left and right:
            return True, ['two-sided finger/object contact detected']
        return False, ['gripper contact incomplete: left=%s right=%s' % (left, right)]

    def _simulate_lift(self, model, seed_data, meta, grasp_q, lift_q, payload):
        data = self._copy_data(model, seed_data)
        self._set_arm_qpos(model, data, meta['arm_joints'], grasp_q)
        self._apply_gripper_width(model, data, float(payload.get('close_width_m', 0.0)))
        self._mujoco.mj_forward(model, data)
        start_z = float(data.xpos[meta['object_body']][2])
        for alpha in np.linspace(0.0, 1.0, 40):
            q = (1.0 - alpha) * np.asarray(grasp_q) + alpha * np.asarray(lift_q)
            self._set_arm_qpos(model, data, meta['arm_joints'], q)
            self._apply_gripper_width(model, data, float(payload.get('close_width_m', 0.0)))
            self._mujoco.mj_step(model, data)
        end_z = float(data.xpos[meta['object_body']][2])
        min_lift = float(payload.get('min_lift_success_m', 0.015))
        if end_z - start_z >= min_lift:
            return True, ['object lifted %.3fm in simulation' % (end_z - start_z)]
        return False, ['object did not lift enough: %.3fm < %.3fm' % (end_z - start_z, min_lift)]

    def _bad_contacts(self, model, data, allow_finger_object=False):
        bad = []
        for index in range(data.ncon):
            contact = data.contact[index]
            first = self._geom_body_name(model, contact.geom1)
            second = self._geom_body_name(model, contact.geom2)
            pair = {first, second}
            if 'target_object' in pair and 'floor' in pair:
                continue
            if allow_finger_object and 'target_object' in pair and (self.left_finger_body in pair or self.right_finger_body in pair):
                continue
            if 'floor' in pair or 'target_object' in pair:
                bad.append('%s/%s' % (first, second))
        return bad

    def _finger_object_contacts(self, model, data, meta):
        left = False
        right = False
        for index in range(data.ncon):
            contact = data.contact[index]
            first = self._geom_body_name(model, contact.geom1)
            second = self._geom_body_name(model, contact.geom2)
            pair = {first, second}
            if 'target_object' not in pair:
                continue
            left = left or self.left_finger_body in pair
            right = right or self.right_finger_body in pair
        return left, right

    def _geom_body_name(self, model, geom_id):
        body_id = int(model.geom_bodyid[int(geom_id)])
        return self._mujoco.mj_id2name(model, self._mujoco.mjtObj.mjOBJ_BODY, body_id) or str(body_id)


def _inject_options(xml):
    if '<option' not in xml:
        return xml.replace('<mujoco', '<mujoco', 1).replace('>', '>\n  <option integrator="implicitfast" cone="elliptic"/>', 1)
    return xml


def _inject_actuators(xml):
    if '<actuator>' in xml:
        return xml
    actuator = """
  <actuator>
    <position name="Joint1_act" joint="Joint1" kp="90" kv="12"/>
    <position name="Joint2_act" joint="Joint2" kp="90" kv="12"/>
    <position name="Joint3_act" joint="Joint3" kp="90" kv="12"/>
    <position name="Joint4_act" joint="Joint4" kp="70" kv="10"/>
    <position name="Joint5_act" joint="Joint5" kp="70" kv="10"/>
    <position name="Joint6_act" joint="Joint6" kp="45" kv="7"/>
    <position name="left_finger_act" joint="left_finger" kp="55" kv="7"/>
    <position name="right_finger_act" joint="right_finger" kp="55" kv="7"/>
  </actuator>"""
    return xml.replace('</mujoco>', actuator + '\n</mujoco>')


def _inject_target_object(xml, object_model):
    object_type = str(object_model.get('type') or 'mouse_compound')
    size = _vector3(object_model.get('size_xyz_m'), [0.10, 0.06, 0.035])
    mass = float(object_model.get('mass_kg', 0.08))
    rgba = object_model.get('rgba') or [0.03, 0.03, 0.035, 1.0]
    rgba_text = ' '.join(str(float(v)) for v in rgba)
    mesh_path = object_model.get('mesh_path')
    asset = ''
    if object_type in ('mesh', 'mouse_mesh') and mesh_path:
        mesh_path = str(Path(mesh_path).expanduser())
        scale = _vector3(object_model.get('mesh_scale'), [1.0, 1.0, 1.0])
        asset = '<mesh name="target_object_mesh" file="%s" scale="%s %s %s"/>' % (
            mesh_path,
            scale[0],
            scale[1],
            scale[2],
        )
        geom = '<geom name="target_object_geom" type="mesh" mesh="target_object_mesh" mass="%s" rgba="%s" friction="1.2 0.08 0.02"/>' % (
            mass,
            rgba_text,
        )
    else:
        half = [max(0.005, float(v) * 0.5) for v in size]
        geom = """
      <geom name="target_mouse_body" type="ellipsoid" size="%.5f %.5f %.5f" mass="%.5f" rgba="%s" friction="1.3 0.08 0.02"/>
      <geom name="target_mouse_nose" type="ellipsoid" pos="%.5f 0 %.5f" size="%.5f %.5f %.5f" mass="%.5f" rgba="%s" friction="1.3 0.08 0.02"/>
      <geom name="target_mouse_wheel" type="cylinder" pos="%.5f 0 %.5f" euler="1.5708 0 0" size="%.5f %.5f" mass="0.002" rgba="0.01 0.01 0.01 1"/>
""" % (
            half[0],
            half[1],
            half[2],
            mass * 0.8,
            rgba_text,
            half[0] * 0.35,
            half[2] * 0.20,
            half[0] * 0.45,
            half[1] * 0.75,
            half[2] * 0.75,
            mass * 0.2,
            rgba_text,
            half[0] * 0.10,
            half[2] * 1.05,
            min(half[1] * 0.18, 0.006),
            min(half[2] * 0.30, 0.006),
        )
    if asset:
        xml = xml.replace('</asset>', '    %s\n  </asset>' % asset)
    body = """
    <body name="target_object" pos="0.30 0 0.04">
      <freejoint name="target_object_joint"/>
%s
    </body>
""" % geom
    return xml.replace('</worldbody>', body + '\n  </worldbody>')


def _parse_grasp_sequence(payload):
    result = []
    for item in payload.get('grasp_sequence_base') or []:
        quat = _normalize_quat(item.get('quaternion_xyzw') or [0.0, 0.0, 0.0, 1.0])
        result.append(
            {
                'name': str(item.get('name') or 'pose'),
                'position': _vector3(item.get('position'), [0.0, 0.0, 0.0]),
                'rotation_matrix': _quat_xyzw_to_matrix(quat),
            }
        )
    return result


def _orientation_error(current, target):
    return 0.5 * (
        np.cross(current[:, 0], target[:, 0])
        + np.cross(current[:, 1], target[:, 1])
        + np.cross(current[:, 2], target[:, 2])
    )


def _quat_xyzw_to_matrix(quat):
    x, y, z, w = _normalize_quat(quat)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.asarray(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def _normalize_quat(value):
    quat = np.asarray(value, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=float)
    quat = quat / norm
    if quat[3] < 0.0:
        quat = -quat
    return quat


def _vector3(value, default):
    if value is None:
        return np.asarray(default, dtype=float)
    vec = np.asarray(value, dtype=float).reshape(3)
    return vec


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description='Alicia WSL GraspNet + MuJoCo digital twin server')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--baseline-root', default=str(Path.home() / 'grasp6d_ws' / 'graspnet-baseline'))
    parser.add_argument('--checkpoint', default=str(Path.home() / 'grasp6d_ws' / 'checkpoints' / 'checkpoint-rs.tar'))
    parser.add_argument('--device', default='cuda:0')
    parser.add_argument('--num-view', type=int, default=300)
    parser.add_argument('--num-points', type=int, default=20000)
    parser.add_argument('--collision-thresh', type=float, default=0.01)
    parser.add_argument('--collision-voxel-size', type=float, default=0.01)
    parser.add_argument('--model-xml', default=str(DEFAULT_MODEL_XML))
    parser.add_argument('--pass-score', type=int, default=80)
    parser.add_argument('--max-joint-state-age-sec', type=float, default=2.0)
    parser.add_argument('--ros-sync-joint-states', action='store_true')
    parser.add_argument('--ros-joint-state-topic', default='/joint_states')
    parser.add_argument('--mock', action='store_true', help='Use mock inference and mock simulation backends')
    parser.add_argument('--mock-graspnet', action='store_true', help='Mock only GraspNet /predict')
    parser.add_argument('--mock-mujoco', action='store_true', help='Mock only MuJoCo /simulate_grasp')
    parser.add_argument('--warmup', action='store_true')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if args.mock or args.mock_graspnet:
        grasp_backend = MockGraspNetBackend()
    else:
        grasp_backend = GraspNetBaselineBackend(
            baseline_root=args.baseline_root,
            checkpoint=args.checkpoint,
            device=args.device,
            num_view=args.num_view,
            num_points=args.num_points,
            collision_thresh=args.collision_thresh,
            collision_voxel_size=args.collision_voxel_size,
        )
    if args.mock or args.mock_mujoco:
        sim_backend = MockDigitalTwinBackend()
    else:
        sim_backend = MujocoDigitalTwinBackend(
            model_xml=args.model_xml,
            pass_score=args.pass_score,
            max_joint_state_age_sec=args.max_joint_state_age_sec,
            ros_sync_joint_states=args.ros_sync_joint_states,
            ros_joint_state_topic=args.ros_joint_state_topic,
        )
    if args.warmup and hasattr(grasp_backend, 'load'):
        grasp_backend.load()
    server = make_server(args.host, args.port, grasp_backend, sim_backend)
    print(
        'Alicia MuJoCo digital twin server listening on http://%s:%d (grasp=%s, sim=%s)'
        % (args.host, args.port, grasp_backend.name, sim_backend.name),
        flush=True,
    )
    server.serve_forever()


if __name__ == '__main__':
    main()
