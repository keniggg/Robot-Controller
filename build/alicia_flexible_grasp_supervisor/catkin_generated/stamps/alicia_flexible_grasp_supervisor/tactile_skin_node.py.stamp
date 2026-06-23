#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, Float32MultiArray, Bool
from alicia_flexible_grasp_supervisor.msg import TactileFrame, TactileState
from alicia_flexible_grasp_supervisor.srv import TriggerZero, TriggerZeroResponse
from alicia_flexible_grasp.tactile.tactile_sdk_wrapper import TactileSdkWrapper
from alicia_flexible_grasp.tactile.tactile_filter import LowPassFilter, split_values, summarize_pressure
from alicia_flexible_grasp.tactile.slip_detector import SlipDetector


class TactileSkinNode:
    def __init__(self):
        def gp(name, default):
            return rospy.get_param('~' + name, rospy.get_param('/tactile/' + name, default))
        self.port = gp('port', '/dev/ttyUSB0')
        self.baudrate = int(gp('baudrate', 4000000))
        self.slave_address = int(gp('slave_address', 1))
        self.timeout = float(gp('timeout', 1.0))
        self.send_wait_secs = float(gp('send_wait_secs', 0.005))
        self.sample_hz = float(gp('sample_hz', 100.0))
        self.use_fast_read = bool(gp('use_fast_read', True))
        self.value_type = int(gp('pressure_value_type', 1))
        self.zero_on_start = bool(gp('trigger_dynamic_zero_on_start', False))
        self.skin1_indices = gp('skin1_indices', list(range(30)))
        self.skin2_indices = gp('skin2_indices', list(range(30, 60)))
        self.rows = int(gp('skin_rows', 5))
        self.cols = int(gp('skin_cols', 6))
        self.contact_threshold = float(gp('contact_threshold_mn', 200.0))
        self.filter1 = LowPassFilter(float(gp('filter_alpha', 0.35)))
        self.filter2 = LowPassFilter(float(gp('filter_alpha', 0.35)))
        self.slip = SlipDetector(float(gp('slip_window_sec', 0.2)), float(gp('slip_drop_ratio', 0.3)))
        self.sdk = None
        self.pub_skin1 = rospy.Publisher('/tactile/skin1/frame', TactileFrame, queue_size=10)
        self.pub_skin2 = rospy.Publisher('/tactile/skin2/frame', TactileFrame, queue_size=10)
        self.pub_state = rospy.Publisher('/tactile/state', TactileState, queue_size=10)
        self.pub_total = rospy.Publisher('/tactile/total_grip_force', Float32, queue_size=10)
        self.pub_diff = rospy.Publisher('/tactile/force_diff', Float32, queue_size=10)
        self.pub_slip = rospy.Publisher('/tactile/slip_state', Bool, queue_size=10)
        self.pub_values = rospy.Publisher('/tactile/raw_values', Float32MultiArray, queue_size=10)
        rospy.Service('/tactile/trigger_zero', TriggerZero, self.handle_zero)

    def handle_zero(self, req):
        try:
            if self.sdk is None:
                return TriggerZeroResponse(False, 'tactile sdk not connected')
            if req.skin_id == 0 or req.skin_id in (1, 2):
                self.sdk.trigger_dynamic_zero()
                self.filter1.reset(); self.filter2.reset(); self.slip.reset()
                return TriggerZeroResponse(True, 'dynamic zero triggered')
            return TriggerZeroResponse(False, 'unsupported skin_id; use 0 for all')
        except Exception as exc:
            return TriggerZeroResponse(False, str(exc))

    def connect(self):
        self.sdk = TactileSdkWrapper(self.port, self.slave_address, self.baudrate, self.timeout, self.send_wait_secs)
        self.sdk.connect()
        self.sdk.set_pressure_value_type(self.value_type)
        if self.zero_on_start:
            self.sdk.trigger_dynamic_zero()
        rospy.loginfo('Tactile skin connected on %s addr=%s', self.port, self.slave_address)

    def make_frame(self, skin_id, values, summary, stamp):
        msg = TactileFrame()
        msg.header.stamp = stamp
        msg.skin_id = skin_id
        msg.rows = self.rows
        msg.cols = self.cols
        msg.values = [float(v) for v in values]
        msg.total_force = summary['total']
        msg.max_force = summary['max_force']
        msg.max_index = summary['max_index']
        msg.center_x = summary['center_x']
        msg.center_y = summary['center_y']
        msg.contact = summary['contact']
        msg.valid = True
        msg.status = 'ok'
        return msg

    def spin(self):
        self.connect()
        rate = rospy.Rate(self.sample_hz)
        while not rospy.is_shutdown():
            try:
                data = self.sdk.read_values(fast=self.use_fast_read)
                if data is None:
                    rate.sleep(); continue
                values, _ = data
                stamp = rospy.Time.now()
                v1 = self.filter1.update(split_values(values, self.skin1_indices))
                v2 = self.filter2.update(split_values(values, self.skin2_indices))
                s1 = summarize_pressure(v1, self.rows, self.cols, self.contact_threshold)
                s2 = summarize_pressure(v2, self.rows, self.cols, self.contact_threshold)
                f1 = self.make_frame(1, v1, s1, stamp)
                f2 = self.make_frame(2, v2, s2, stamp)
                total = s1['total'] + s2['total']
                diff = s1['total'] - s2['total']
                slip = self.slip.update(stamp.to_sec(), total)
                st = TactileState()
                st.header.stamp = stamp
                st.skin1 = f1; st.skin2 = f2
                st.total_grip_force = total
                st.force_diff = diff
                st.left_contact = s1['contact']
                st.right_contact = s2['contact']
                st.object_grasped = s1['contact'] and s2['contact']
                st.slip_detected = slip
                st.status = 'ok'
                self.pub_skin1.publish(f1); self.pub_skin2.publish(f2); self.pub_state.publish(st)
                self.pub_total.publish(Float32(data=total)); self.pub_diff.publish(Float32(data=diff)); self.pub_slip.publish(Bool(data=slip))
                self.pub_values.publish(Float32MultiArray(data=[float(v) for v in values]))
            except Exception as exc:
                rospy.logwarn_throttle(1.0, 'tactile read error: %s', exc)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tactile_skin_node')
    TactileSkinNode().spin()
