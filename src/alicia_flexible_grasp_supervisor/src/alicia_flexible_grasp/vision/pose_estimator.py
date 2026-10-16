from alicia_flexible_grasp.utils.transform_utils import transform_point, make_pose_stamped, transform_pose

class PoseEstimator:
    def __init__(
        self,
        camera_frame,
        base_frame,
        translation_xyz,
        rotation_xyzw,
        default_orientation_xyzw,
        tf_buffer=None,
        tf_timeout_sec=0.2,
        tf_lookup_latest=True,
        allow_static_fallback=True,
    ):
        self.camera_frame = camera_frame
        self.base_frame = base_frame
        self.translation_xyz = translation_xyz
        self.rotation_xyzw = rotation_xyzw
        self.default_orientation_xyzw = default_orientation_xyzw
        self.tf_buffer = tf_buffer
        self.tf_timeout_sec = max(0.0, float(tf_timeout_sec))
        self.tf_lookup_latest = bool(tf_lookup_latest)
        self.allow_static_fallback = bool(allow_static_fallback)
        self.last_transform_source = 'none'

    def make_poses(self, p_camera_xyz, stamp=None, camera_frame=None):
        source_frame = camera_frame or self.camera_frame
        p_base_xyz = self._camera_point_to_base(p_camera_xyz, source_frame, stamp)
        pose_cam = make_pose_stamped(source_frame, p_camera_xyz, self.default_orientation_xyzw, stamp)
        pose_base = make_pose_stamped(self.base_frame, p_base_xyz, self.default_orientation_xyzw, stamp)
        return pose_cam, pose_base

    def make_base_pose_from_camera_pose(self, p_camera_xyz, q_camera_xyzw, stamp=None, camera_frame=None):
        source_frame = camera_frame or self.camera_frame
        p_base_xyz, q_base_xyzw = self._camera_pose_to_base(
            p_camera_xyz,
            q_camera_xyzw,
            source_frame,
            stamp,
        )
        return make_pose_stamped(self.base_frame, p_base_xyz, q_base_xyzw, stamp)

    def _camera_point_to_base(self, p_camera_xyz, source_frame, stamp):
        if self.tf_buffer is not None:
            try:
                transform = self._lookup_transform(source_frame, stamp)
                return self._transform_point_with_tf(p_camera_xyz, transform)
            except Exception as exc:
                self._warn_tf_fallback(source_frame, exc)
        if not self.allow_static_fallback:
            raise RuntimeError('TF lookup failed and static handeye fallback is disabled')
        self.last_transform_source = 'static_handeye'
        return transform_point(p_camera_xyz, self.translation_xyz, self.rotation_xyzw)

    def _camera_pose_to_base(self, p_camera_xyz, q_camera_xyzw, source_frame, stamp):
        if self.tf_buffer is not None:
            try:
                transform = self._lookup_transform(source_frame, stamp)
                return self._transform_pose_with_tf(p_camera_xyz, q_camera_xyzw, transform)
            except Exception as exc:
                self._warn_tf_fallback(source_frame, exc)
        if not self.allow_static_fallback:
            raise RuntimeError('TF lookup failed and static handeye fallback is disabled')
        self.last_transform_source = 'static_handeye'
        return transform_pose(p_camera_xyz, q_camera_xyzw, self.translation_xyz, self.rotation_xyzw)

    def _lookup_transform(self, source_frame, stamp):
        import rospy
        lookup_time = rospy.Time(0) if self.tf_lookup_latest or stamp is None else stamp
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                lookup_time,
                rospy.Duration(self.tf_timeout_sec),
            )
            self.last_transform_source = 'tf_latest' if lookup_time == rospy.Time(0) else 'tf_stamp'
            return transform
        except Exception:
            if lookup_time == rospy.Time(0):
                raise
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout_sec),
            )
            self.last_transform_source = 'tf_latest_fallback'
            return transform

    def _warn_tf_fallback(self, source_frame, exc):
        try:
            import rospy
            rospy.logwarn_throttle(
                2.0,
                'TF lookup %s -> %s failed, falling back to configured handeye transform: %s',
                source_frame,
                self.base_frame,
                exc,
            )
        except Exception:
            pass

    @staticmethod
    def _transform_point_with_tf(point_xyz, transform):
        trans = transform.transform.translation
        rot = transform.transform.rotation
        return transform_point(
            point_xyz,
            [trans.x, trans.y, trans.z],
            [rot.x, rot.y, rot.z, rot.w],
        )

    @staticmethod
    def _transform_pose_with_tf(point_xyz, quat_xyzw, transform):
        trans = transform.transform.translation
        rot = transform.transform.rotation
        return transform_pose(
            point_xyz,
            quat_xyzw,
            [trans.x, trans.y, trans.z],
            [rot.x, rot.y, rot.z, rot.w],
        )
