#!/usr/bin/env python3
import datetime
import json
import math
import os
import threading
from collections import deque

import numpy as np
import rospy
import tf2_ros
import yaml
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf.transformations import quaternion_matrix

from alicia_flexible_grasp.calibration.tcp_solver import (
    TcpCalibrationError,
    max_orientation_separation_rad,
    rotation_distance_rad,
    solve_tcp_pivot,
)
from alicia_flexible_grasp_supervisor.srv import TcpCalibrationCommand, TcpCalibrationCommandResponse


class TcpCalibrationNode:
    def __init__(self):
        cfg = rospy.get_param('/tcp_calibration', {})
        self.base_frame = str(cfg.get('base_frame', 'base_link'))
        self.reference_frame = str(cfg.get('reference_frame', 'Grasp_base'))
        self.result_directory = os.path.expanduser(str(cfg.get('result_directory', '~/alicia_wa_full/calibration_results')))
        self.min_samples = int(cfg.get('min_samples', 8))
        self.min_orientation_separation_rad = math.radians(float(cfg.get('min_orientation_separation_deg', 20.0)))
        self.min_new_sample_separation_rad = math.radians(float(cfg.get('min_new_sample_separation_deg', 5.0)))
        self.joint_state_max_age_sec = float(cfg.get('joint_state_max_age_sec', 1.0))
        self.settle_window_sec = float(cfg.get('settle_window_sec', 0.4))
        self.settle_max_joint_range_rad = float(cfg.get('settle_max_joint_range_rad', 0.003))
        self.tf_timeout_sec = float(cfg.get('tf_timeout_sec', 0.3))
        self.good_rms_error_m = float(cfg.get('good_rms_error_m', 0.003))
        self.good_max_error_m = float(cfg.get('good_max_error_m', 0.005))

        self._lock = threading.RLock()
        self.samples = []
        self.solution = None
        self.last_result_file = ''
        self.joint_history = deque(maxlen=200)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        topic = str(cfg.get('joint_state_topic', '/joint_states'))
        self.joint_subscriber = rospy.Subscriber(topic, JointState, self._joint_callback, queue_size=20)
        self.status_publisher = rospy.Publisher('/tcp_calibration/status', String, queue_size=1, latch=True)
        self.service = rospy.Service('/tcp_calibration/command', TcpCalibrationCommand, self._handle_command)
        self._publish_status('READY', '等待固定点 TCP 采样')
        rospy.loginfo(
            'TCP calibration ready: %s -> %s, min_samples=%d',
            self.base_frame,
            self.reference_frame,
            self.min_samples,
        )

    def _joint_callback(self, msg):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp and msg.header.stamp.to_sec() > 0.0 else rospy.get_time()
        arm_positions = tuple(float(value) for value in list(msg.position)[:6])
        if len(arm_positions) == 6 and all(math.isfinite(value) for value in arm_positions):
            with self._lock:
                self.joint_history.append((stamp, arm_positions))

    def _handle_command(self, request):
        command = str(request.command or '').strip().lower()
        try:
            if command == 'capture':
                return self._capture_sample()
            if command == 'undo':
                return self._undo_sample()
            if command == 'clear':
                return self._clear_samples()
            if command == 'solve':
                return self._solve()
            if command == 'save':
                return self._save()
            if command == 'status':
                return self._response(True, 'status')
            return self._response(False, 'unknown command: %s' % command)
        except Exception as exc:
            rospy.logerr('TCP calibration command %s failed: %s', command, exc)
            self._publish_status('ERROR', str(exc))
            return self._response(False, str(exc))

    def _capture_sample(self):
        stable, reason = self._joint_feedback_is_stable()
        if not stable:
            return self._response(False, reason)
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            self.reference_frame,
            rospy.Time(0),
            rospy.Duration(self.tf_timeout_sec),
        )
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=float)
        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ], dtype=float)
        rotation = quaternion_matrix(quaternion)[:3, :3]

        with self._lock:
            if self.samples:
                separation = min(rotation_distance_rad(rotation, item['rotation']) for item in self.samples)
                if separation < self.min_new_sample_separation_rad:
                    return self._response(
                        False,
                        '新姿态与已有样本最小夹角仅 %.1f deg，请改变腕部姿态后再采样'
                        % math.degrees(separation),
                    )
            sample = {
                'translation': translation,
                'rotation': rotation,
                'quaternion_xyzw': quaternion,
                'stamp': rospy.get_time(),
            }
            self.samples.append(sample)
            self.solution = None
            count = len(self.samples)
            span = math.degrees(max_orientation_separation_rad(self.samples))
        message = '已采样 %d 个姿态，当前姿态跨度 %.1f deg' % (count, span)
        rospy.loginfo('TCP sample %d: %s->%s xyz=%s span=%.1fdeg', count, self.base_frame, self.reference_frame, translation, span)
        self._publish_status('CAPTURED', message)
        return self._response(True, message)

    def _joint_feedback_is_stable(self):
        now = rospy.get_time()
        with self._lock:
            history = list(self.joint_history)
        if not history:
            return False, '没有收到 /joint_states，禁止采样'
        if now - history[-1][0] > self.joint_state_max_age_sec:
            return False, '关节反馈已过期，禁止采样'
        recent = [(stamp, positions) for stamp, positions in history if now - stamp <= self.settle_window_sec]
        if len(recent) < 3 or recent[-1][0] - recent[0][0] < self.settle_window_sec * 0.7:
            return False, '等待机械臂停稳后再采样'
        window = [positions for _, positions in recent]
        ranges = np.ptp(np.asarray(window, dtype=float), axis=0)
        if float(np.max(ranges)) > self.settle_max_joint_range_rad:
            return False, '机械臂仍在运动，最大关节变化 %.4f rad' % float(np.max(ranges))
        return True, 'stable'

    def _undo_sample(self):
        with self._lock:
            if not self.samples:
                return self._response(False, '当前没有可撤销的样本')
            self.samples.pop()
            self.solution = None
            count = len(self.samples)
        message = '已撤销最后一个样本，剩余 %d 个' % count
        self._publish_status('READY', message)
        return self._response(True, message)

    def _clear_samples(self):
        with self._lock:
            self.samples = []
            self.solution = None
            self.last_result_file = ''
        message = '样本已清空'
        self._publish_status('READY', message)
        return self._response(True, message)

    def _solve(self):
        with self._lock:
            samples = list(self.samples)
        try:
            solution = solve_tcp_pivot(
                samples,
                min_samples=self.min_samples,
                min_orientation_separation_rad=self.min_orientation_separation_rad,
            )
        except TcpCalibrationError as exc:
            message = '求解失败：%s' % exc
            self._publish_status('NEED_MORE_POSES', message)
            return self._response(False, message)
        with self._lock:
            self.solution = solution
        quality = 'PASS' if solution['rms_error'] <= self.good_rms_error_m and solution['max_error'] <= self.good_max_error_m else 'CHECK'
        message = (
            '求解%s：TCP=(%.4f, %.4f, %.4f)m RMS=%.2fmm MAX=%.2fmm 姿态跨度=%.1fdeg'
            % (
                quality,
                solution['tcp_translation'][0],
                solution['tcp_translation'][1],
                solution['tcp_translation'][2],
                solution['rms_error'] * 1000.0,
                solution['max_error'] * 1000.0,
                math.degrees(solution['orientation_span_rad']),
            )
        )
        rospy.loginfo(message)
        self._publish_status('SOLVED_' + quality, message)
        return self._response(True, message)

    def _save(self):
        with self._lock:
            solution = self.solution
            samples = list(self.samples)
        if solution is None:
            return self._response(False, '请先完成求解再保存')
        os.makedirs(self.result_directory, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self.result_directory, 'tcp_calibration_%s.yaml' % stamp)
        payload = {
            'tcp_calibration': {
                'base_frame': self.base_frame,
                'reference_frame': self.reference_frame,
                'tcp_translation_xyz_m': [float(value) for value in solution['tcp_translation']],
                'fixed_point_xyz_m': [float(value) for value in solution['fixed_point']],
                'rms_error_m': float(solution['rms_error']),
                'max_error_m': float(solution['max_error']),
                'orientation_span_deg': math.degrees(solution['orientation_span_rad']),
                'condition': float(solution['condition']),
                'sample_count': len(samples),
                'samples': [
                    {
                        'reference_translation_xyz_m': [float(value) for value in item['translation']],
                        'reference_quaternion_xyzw': [float(value) for value in item['quaternion_xyzw']],
                    }
                    for item in samples
                ],
            }
        }
        with open(path, 'w') as stream:
            yaml.safe_dump(payload, stream, default_flow_style=False, sort_keys=False)
        with self._lock:
            self.last_result_file = path
        message = '标定结果已保存：%s（尚未写入 URDF）' % path
        rospy.loginfo(message)
        self._publish_status('SAVED', message)
        return self._response(True, message)

    def _response(self, success, message):
        response = TcpCalibrationCommandResponse()
        response.success = bool(success)
        response.message = str(message)
        with self._lock:
            response.sample_count = len(self.samples)
            solution = self.solution
            response.result_file = self.last_result_file
        if solution is not None:
            response.tcp_translation.x, response.tcp_translation.y, response.tcp_translation.z = [float(v) for v in solution['tcp_translation']]
            response.fixed_point.x, response.fixed_point.y, response.fixed_point.z = [float(v) for v in solution['fixed_point']]
            response.rms_error_m = float(solution['rms_error'])
            response.max_error_m = float(solution['max_error'])
            response.orientation_span_deg = math.degrees(solution['orientation_span_rad'])
        return response

    def _publish_status(self, state, message):
        with self._lock:
            solution = self.solution
            payload = {
                'state': str(state),
                'message': str(message),
                'sample_count': len(self.samples),
                'minimum_samples': self.min_samples,
                'base_frame': self.base_frame,
                'reference_frame': self.reference_frame,
                'result_file': self.last_result_file,
            }
        if solution is not None:
            payload.update({
                'tcp_translation_xyz_m': [float(v) for v in solution['tcp_translation']],
                'rms_error_m': float(solution['rms_error']),
                'max_error_m': float(solution['max_error']),
                'orientation_span_deg': math.degrees(solution['orientation_span_rad']),
            })
        self.status_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main():
    rospy.init_node('tcp_calibration_node')
    TcpCalibrationNode()
    rospy.spin()


if __name__ == '__main__':
    main()
