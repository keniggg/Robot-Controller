;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::ObjectGeometry)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'ObjectGeometry (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::OBJECTGEOMETRY")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::OBJECTGEOMETRY"))

(in-package "ROS")
;;//! \htmlinclude ObjectGeometry.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass alicia_flexible_grasp_supervisor::ObjectGeometry
  :super ros::object
  :slots (_header _valid _label _source_mode _pose_base _size_xyz_m _support_normal_base _support_offset_m _valid_depth_points _valid_depth_ratio _depth_mad_m _fused_frames _support_inlier_ratio _object_point_count _failure_reason ))

(defmethod alicia_flexible_grasp_supervisor::ObjectGeometry
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:valid __valid) nil)
    ((:label __label) "")
    ((:source_mode __source_mode) "")
    ((:pose_base __pose_base) (instance geometry_msgs::Pose :init))
    ((:size_xyz_m __size_xyz_m) (instance geometry_msgs::Vector3 :init))
    ((:support_normal_base __support_normal_base) (instance geometry_msgs::Vector3 :init))
    ((:support_offset_m __support_offset_m) 0.0)
    ((:valid_depth_points __valid_depth_points) 0)
    ((:valid_depth_ratio __valid_depth_ratio) 0.0)
    ((:depth_mad_m __depth_mad_m) 0.0)
    ((:fused_frames __fused_frames) 0)
    ((:support_inlier_ratio __support_inlier_ratio) 0.0)
    ((:object_point_count __object_point_count) 0)
    ((:failure_reason __failure_reason) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _valid __valid)
   (setq _label (string __label))
   (setq _source_mode (string __source_mode))
   (setq _pose_base __pose_base)
   (setq _size_xyz_m __size_xyz_m)
   (setq _support_normal_base __support_normal_base)
   (setq _support_offset_m (float __support_offset_m))
   (setq _valid_depth_points (round __valid_depth_points))
   (setq _valid_depth_ratio (float __valid_depth_ratio))
   (setq _depth_mad_m (float __depth_mad_m))
   (setq _fused_frames (round __fused_frames))
   (setq _support_inlier_ratio (float __support_inlier_ratio))
   (setq _object_point_count (round __object_point_count))
   (setq _failure_reason (string __failure_reason))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:source_mode
   (&optional __source_mode)
   (if __source_mode (setq _source_mode __source_mode)) _source_mode)
  (:pose_base
   (&rest __pose_base)
   (if (keywordp (car __pose_base))
       (send* _pose_base __pose_base)
     (progn
       (if __pose_base (setq _pose_base (car __pose_base)))
       _pose_base)))
  (:size_xyz_m
   (&rest __size_xyz_m)
   (if (keywordp (car __size_xyz_m))
       (send* _size_xyz_m __size_xyz_m)
     (progn
       (if __size_xyz_m (setq _size_xyz_m (car __size_xyz_m)))
       _size_xyz_m)))
  (:support_normal_base
   (&rest __support_normal_base)
   (if (keywordp (car __support_normal_base))
       (send* _support_normal_base __support_normal_base)
     (progn
       (if __support_normal_base (setq _support_normal_base (car __support_normal_base)))
       _support_normal_base)))
  (:support_offset_m
   (&optional __support_offset_m)
   (if __support_offset_m (setq _support_offset_m __support_offset_m)) _support_offset_m)
  (:valid_depth_points
   (&optional __valid_depth_points)
   (if __valid_depth_points (setq _valid_depth_points __valid_depth_points)) _valid_depth_points)
  (:valid_depth_ratio
   (&optional __valid_depth_ratio)
   (if __valid_depth_ratio (setq _valid_depth_ratio __valid_depth_ratio)) _valid_depth_ratio)
  (:depth_mad_m
   (&optional __depth_mad_m)
   (if __depth_mad_m (setq _depth_mad_m __depth_mad_m)) _depth_mad_m)
  (:fused_frames
   (&optional __fused_frames)
   (if __fused_frames (setq _fused_frames __fused_frames)) _fused_frames)
  (:support_inlier_ratio
   (&optional __support_inlier_ratio)
   (if __support_inlier_ratio (setq _support_inlier_ratio __support_inlier_ratio)) _support_inlier_ratio)
  (:object_point_count
   (&optional __object_point_count)
   (if __object_point_count (setq _object_point_count __object_point_count)) _object_point_count)
  (:failure_reason
   (&optional __failure_reason)
   (if __failure_reason (setq _failure_reason __failure_reason)) _failure_reason)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _valid
    1
    ;; string _label
    4 (length _label)
    ;; string _source_mode
    4 (length _source_mode)
    ;; geometry_msgs/Pose _pose_base
    (send _pose_base :serialization-length)
    ;; geometry_msgs/Vector3 _size_xyz_m
    (send _size_xyz_m :serialization-length)
    ;; geometry_msgs/Vector3 _support_normal_base
    (send _support_normal_base :serialization-length)
    ;; float32 _support_offset_m
    4
    ;; uint32 _valid_depth_points
    4
    ;; float32 _valid_depth_ratio
    4
    ;; float32 _depth_mad_m
    4
    ;; uint32 _fused_frames
    4
    ;; float32 _support_inlier_ratio
    4
    ;; uint32 _object_point_count
    4
    ;; string _failure_reason
    4 (length _failure_reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; string _source_mode
       (write-long (length _source_mode) s) (princ _source_mode s)
     ;; geometry_msgs/Pose _pose_base
       (send _pose_base :serialize s)
     ;; geometry_msgs/Vector3 _size_xyz_m
       (send _size_xyz_m :serialize s)
     ;; geometry_msgs/Vector3 _support_normal_base
       (send _support_normal_base :serialize s)
     ;; float32 _support_offset_m
       (sys::poke _support_offset_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _valid_depth_points
       (write-long _valid_depth_points s)
     ;; float32 _valid_depth_ratio
       (sys::poke _valid_depth_ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _depth_mad_m
       (sys::poke _depth_mad_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _fused_frames
       (write-long _fused_frames s)
     ;; float32 _support_inlier_ratio
       (sys::poke _support_inlier_ratio (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _object_point_count
       (write-long _object_point_count s)
     ;; string _failure_reason
       (write-long (length _failure_reason) s) (princ _failure_reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _source_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _source_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Pose _pose_base
     (send _pose_base :deserialize buf ptr-) (incf ptr- (send _pose_base :serialization-length))
   ;; geometry_msgs/Vector3 _size_xyz_m
     (send _size_xyz_m :deserialize buf ptr-) (incf ptr- (send _size_xyz_m :serialization-length))
   ;; geometry_msgs/Vector3 _support_normal_base
     (send _support_normal_base :deserialize buf ptr-) (incf ptr- (send _support_normal_base :serialization-length))
   ;; float32 _support_offset_m
     (setq _support_offset_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _valid_depth_points
     (setq _valid_depth_points (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _valid_depth_ratio
     (setq _valid_depth_ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _depth_mad_m
     (setq _depth_mad_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _fused_frames
     (setq _fused_frames (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _support_inlier_ratio
     (setq _support_inlier_ratio (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _object_point_count
     (setq _object_point_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _failure_reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _failure_reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::ObjectGeometry :md5sum-) "be6e291b8f028ecb31c9e9f8f891dc32")
(setf (get alicia_flexible_grasp_supervisor::ObjectGeometry :datatype-) "alicia_flexible_grasp_supervisor/ObjectGeometry")
(setf (get alicia_flexible_grasp_supervisor::ObjectGeometry :definition-)
      "std_msgs/Header header
bool valid
string label
string source_mode
geometry_msgs/Pose pose_base
geometry_msgs/Vector3 size_xyz_m
geometry_msgs/Vector3 support_normal_base
float32 support_offset_m
uint32 valid_depth_points
float32 valid_depth_ratio
float32 depth_mad_m
uint32 fused_frames
float32 support_inlier_ratio
uint32 object_point_count
string failure_reason

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :alicia_flexible_grasp_supervisor/ObjectGeometry "be6e291b8f028ecb31c9e9f8f891dc32")


