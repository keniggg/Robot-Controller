#!/usr/bin/env python3
import rospy
from alicia_flexible_grasp_supervisor.msg import TactileFrame, TactileState
from alicia_flexible_grasp.tactile.tactile_sdk_wrapper import TactileSDKWrapper
from alicia_flexible_grasp.tactile.tactile_filter import LowPassArray, split_values, contact_center, SlipDetector

class TactileSkinNode:
    def __init__(self):
        cfg = rospy.get_param('/tactile', {})
        self.rows = int(cfg.get('rows', 5))
        self.cols = int(cfg.get('cols', 12))
        self.split_mode = cfg.get('split_mode', 'half')
        self.threshold = float(cfg.get('contact_threshold_mn', 200.0))
        self.filter = LowPassArray(cfg.get('lowpass_alpha', 0.35))
        self.slip = SlipDetector(cfg.get('slip_window_sec', 0.25), cfg.get('slip_drop_ratio', 0.30))
        self.wrapper = TactileSDKWrapper(
            port=cfg.get('port', '/dev/ttyACM0'),
            slave_address=cfg.get('slave_address', 1),
            baudrate=cfg.get('baudrate', 4000000),
            sdk_path=cfg.get('sdk_path', None),
            pressure_value_type=cfg.get('pressure_value_type', 1),
            dynamic_zero_on_start=cfg.get('dynamic_zero_on_start', True),
            simulate=cfg.get('simulate', False),
        )
        self.pub_state = rospy.Publisher('/tactile/state', TactileState, queue_size=10)
        self.pub_left = rospy.Publisher('/tactile/skin1/frame', TactileFrame, queue_size=10)
        self.pub_right = rospy.Publisher('/tactile/skin2/frame', TactileFrame, queue_size=10)
        self.connected = False
        try:
            self.wrapper.connect()
            self.connected = True
            rospy.loginfo('Tactile SDK connected')
        except Exception as exc:
            rospy.logerr('Tactile connect failed: %s. Continue in invalid mode.', exc)
        self.rate_hz = float(cfg.get('read_hz', 120))

    def make_frame(self, name, values, stamp):
        f = TactileFrame()
        f.header.stamp = stamp
        f.header.frame_id = name
        f.skin_name = name
        f.values = [float(v) for v in values]
        f.rows = self.rows
        f.cols = self.cols if len(values) == self.rows*self.cols else max(1, int(len(values)/max(1,self.rows)))
        f.total_force_mn = float(sum(values))
        cx, cy, max_idx, max_val = contact_center(list(values), self.rows, self.cols)
        f.max_force_mn = float(max_val)
        f.max_index = int(max_idx)
        f.center_x = float(cx)
        f.center_y = float(cy)
        f.contact = f.total_force_mn >= self.threshold
        f.valid = self.connected
        return f

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            raw = self.wrapper.read_values()
            if raw is None:
                rate.sleep(); continue
            values = self.filter.update(raw)
            left_vals, right_vals = split_values(values, self.split_mode)
            if self.split_mode == 'single':
                right_vals = []
            stamp = rospy.Time.now()
            left = self.make_frame('skin1', left_vals, stamp)
            right = self.make_frame('skin2', right_vals, stamp) if right_vals else TactileFrame()
            if not right_vals:
                right.header.stamp = stamp; right.skin_name='skin2'; right.valid=False
            st = TactileState()
            st.header.stamp = stamp
            st.left = left
            st.right = right
            st.total_grip_force_mn = left.total_force_mn + right.total_force_mn
            st.force_diff_mn = left.total_force_mn - right.total_force_mn
            st.left_contact = left.contact
            st.right_contact = right.contact
            st.object_grasped = st.total_grip_force_mn >= self.threshold
            st.slip_detected = self.slip.update(st.total_grip_force_mn)
            st.valid = self.connected
            self.pub_left.publish(left)
            if right_vals:
                self.pub_right.publish(right)
            self.pub_state.publish(st)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('tactile_skin_node')
    node = TactileSkinNode()
    rospy.on_shutdown(node.wrapper.close)
    node.spin()
