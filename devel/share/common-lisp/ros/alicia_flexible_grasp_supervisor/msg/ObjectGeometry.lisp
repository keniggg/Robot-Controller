; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-msg)


;//! \htmlinclude ObjectGeometry.msg.html

(cl:defclass <ObjectGeometry> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil)
   (label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (source_mode
    :reader source_mode
    :initarg :source_mode
    :type cl:string
    :initform "")
   (pose_base
    :reader pose_base
    :initarg :pose_base
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (size_xyz_m
    :reader size_xyz_m
    :initarg :size_xyz_m
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (support_normal_base
    :reader support_normal_base
    :initarg :support_normal_base
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (support_offset_m
    :reader support_offset_m
    :initarg :support_offset_m
    :type cl:float
    :initform 0.0)
   (valid_depth_points
    :reader valid_depth_points
    :initarg :valid_depth_points
    :type cl:integer
    :initform 0)
   (valid_depth_ratio
    :reader valid_depth_ratio
    :initarg :valid_depth_ratio
    :type cl:float
    :initform 0.0)
   (depth_mad_m
    :reader depth_mad_m
    :initarg :depth_mad_m
    :type cl:float
    :initform 0.0)
   (fused_frames
    :reader fused_frames
    :initarg :fused_frames
    :type cl:integer
    :initform 0)
   (support_inlier_ratio
    :reader support_inlier_ratio
    :initarg :support_inlier_ratio
    :type cl:float
    :initform 0.0)
   (object_point_count
    :reader object_point_count
    :initarg :object_point_count
    :type cl:integer
    :initform 0)
   (failure_reason
    :reader failure_reason
    :initarg :failure_reason
    :type cl:string
    :initform ""))
)

(cl:defclass ObjectGeometry (<ObjectGeometry>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectGeometry>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectGeometry)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-msg:<ObjectGeometry> is deprecated: use alicia_flexible_grasp_supervisor-msg:ObjectGeometry instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:header-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:valid-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:valid instead.")
  (valid m))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:label-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'source_mode-val :lambda-list '(m))
(cl:defmethod source_mode-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:source_mode-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:source_mode instead.")
  (source_mode m))

(cl:ensure-generic-function 'pose_base-val :lambda-list '(m))
(cl:defmethod pose_base-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:pose_base-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:pose_base instead.")
  (pose_base m))

(cl:ensure-generic-function 'size_xyz_m-val :lambda-list '(m))
(cl:defmethod size_xyz_m-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:size_xyz_m-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:size_xyz_m instead.")
  (size_xyz_m m))

(cl:ensure-generic-function 'support_normal_base-val :lambda-list '(m))
(cl:defmethod support_normal_base-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:support_normal_base-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:support_normal_base instead.")
  (support_normal_base m))

(cl:ensure-generic-function 'support_offset_m-val :lambda-list '(m))
(cl:defmethod support_offset_m-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:support_offset_m-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:support_offset_m instead.")
  (support_offset_m m))

(cl:ensure-generic-function 'valid_depth_points-val :lambda-list '(m))
(cl:defmethod valid_depth_points-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:valid_depth_points-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:valid_depth_points instead.")
  (valid_depth_points m))

(cl:ensure-generic-function 'valid_depth_ratio-val :lambda-list '(m))
(cl:defmethod valid_depth_ratio-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:valid_depth_ratio-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:valid_depth_ratio instead.")
  (valid_depth_ratio m))

(cl:ensure-generic-function 'depth_mad_m-val :lambda-list '(m))
(cl:defmethod depth_mad_m-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:depth_mad_m-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:depth_mad_m instead.")
  (depth_mad_m m))

(cl:ensure-generic-function 'fused_frames-val :lambda-list '(m))
(cl:defmethod fused_frames-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:fused_frames-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:fused_frames instead.")
  (fused_frames m))

(cl:ensure-generic-function 'support_inlier_ratio-val :lambda-list '(m))
(cl:defmethod support_inlier_ratio-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:support_inlier_ratio-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:support_inlier_ratio instead.")
  (support_inlier_ratio m))

(cl:ensure-generic-function 'object_point_count-val :lambda-list '(m))
(cl:defmethod object_point_count-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:object_point_count-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:object_point_count instead.")
  (object_point_count m))

(cl:ensure-generic-function 'failure_reason-val :lambda-list '(m))
(cl:defmethod failure_reason-val ((m <ObjectGeometry>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:failure_reason-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:failure_reason instead.")
  (failure_reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectGeometry>) ostream)
  "Serializes a message object of type '<ObjectGeometry>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'source_mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'source_mode))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_base) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'size_xyz_m) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'support_normal_base) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'support_offset_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'valid_depth_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'valid_depth_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'valid_depth_points)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'valid_depth_points)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'valid_depth_ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth_mad_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fused_frames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fused_frames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fused_frames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fused_frames)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'support_inlier_ratio))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_point_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_point_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_point_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_point_count)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'failure_reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'failure_reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectGeometry>) istream)
  "Deserializes a message object of type '<ObjectGeometry>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'source_mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'source_mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_base) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'size_xyz_m) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'support_normal_base) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'support_offset_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'valid_depth_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'valid_depth_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'valid_depth_points)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'valid_depth_points)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'valid_depth_ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth_mad_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'fused_frames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'fused_frames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'fused_frames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'fused_frames)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'support_inlier_ratio) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'object_point_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'object_point_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'object_point_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'object_point_count)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'failure_reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'failure_reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectGeometry>)))
  "Returns string type for a message object of type '<ObjectGeometry>"
  "alicia_flexible_grasp_supervisor/ObjectGeometry")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectGeometry)))
  "Returns string type for a message object of type 'ObjectGeometry"
  "alicia_flexible_grasp_supervisor/ObjectGeometry")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectGeometry>)))
  "Returns md5sum for a message object of type '<ObjectGeometry>"
  "be6e291b8f028ecb31c9e9f8f891dc32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectGeometry)))
  "Returns md5sum for a message object of type 'ObjectGeometry"
  "be6e291b8f028ecb31c9e9f8f891dc32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectGeometry>)))
  "Returns full string definition for message of type '<ObjectGeometry>"
  (cl:format cl:nil "std_msgs/Header header~%bool valid~%string label~%string source_mode~%geometry_msgs/Pose pose_base~%geometry_msgs/Vector3 size_xyz_m~%geometry_msgs/Vector3 support_normal_base~%float32 support_offset_m~%uint32 valid_depth_points~%float32 valid_depth_ratio~%float32 depth_mad_m~%uint32 fused_frames~%float32 support_inlier_ratio~%uint32 object_point_count~%string failure_reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectGeometry)))
  "Returns full string definition for message of type 'ObjectGeometry"
  (cl:format cl:nil "std_msgs/Header header~%bool valid~%string label~%string source_mode~%geometry_msgs/Pose pose_base~%geometry_msgs/Vector3 size_xyz_m~%geometry_msgs/Vector3 support_normal_base~%float32 support_offset_m~%uint32 valid_depth_points~%float32 valid_depth_ratio~%float32 depth_mad_m~%uint32 fused_frames~%float32 support_inlier_ratio~%uint32 object_point_count~%string failure_reason~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectGeometry>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'label))
     4 (cl:length (cl:slot-value msg 'source_mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_base))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'size_xyz_m))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'support_normal_base))
     4
     4
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'failure_reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectGeometry>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectGeometry
    (cl:cons ':header (header msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':label (label msg))
    (cl:cons ':source_mode (source_mode msg))
    (cl:cons ':pose_base (pose_base msg))
    (cl:cons ':size_xyz_m (size_xyz_m msg))
    (cl:cons ':support_normal_base (support_normal_base msg))
    (cl:cons ':support_offset_m (support_offset_m msg))
    (cl:cons ':valid_depth_points (valid_depth_points msg))
    (cl:cons ':valid_depth_ratio (valid_depth_ratio msg))
    (cl:cons ':depth_mad_m (depth_mad_m msg))
    (cl:cons ':fused_frames (fused_frames msg))
    (cl:cons ':support_inlier_ratio (support_inlier_ratio msg))
    (cl:cons ':object_point_count (object_point_count msg))
    (cl:cons ':failure_reason (failure_reason msg))
))
