; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-msg)


;//! \htmlinclude Grasp6DPlan.msg.html

(cl:defclass <Grasp6DPlan> (roslisp-msg-protocol:ros-message)
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
   (poses
    :reader poses
    :initarg :poses
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (score
    :reader score
    :initarg :score
    :type cl:float
    :initform 0.0)
   (candidate_width_m
    :reader candidate_width_m
    :initarg :candidate_width_m
    :type cl:float
    :initform 0.0)
   (required_open_width_m
    :reader required_open_width_m
    :initarg :required_open_width_m
    :type cl:float
    :initform 0.0)
   (object_geometry
    :reader object_geometry
    :initarg :object_geometry
    :type alicia_flexible_grasp_supervisor-msg:ObjectGeometry
    :initform (cl:make-instance 'alicia_flexible_grasp_supervisor-msg:ObjectGeometry))
   (model_choice
    :reader model_choice
    :initarg :model_choice
    :type cl:string
    :initform "")
   (plan_id
    :reader plan_id
    :initarg :plan_id
    :type cl:string
    :initform "")
   (diagnostic
    :reader diagnostic
    :initarg :diagnostic
    :type cl:string
    :initform ""))
)

(cl:defclass Grasp6DPlan (<Grasp6DPlan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Grasp6DPlan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Grasp6DPlan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-msg:<Grasp6DPlan> is deprecated: use alicia_flexible_grasp_supervisor-msg:Grasp6DPlan instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:header-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:valid-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:valid instead.")
  (valid m))

(cl:ensure-generic-function 'poses-val :lambda-list '(m))
(cl:defmethod poses-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:poses-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:poses instead.")
  (poses m))

(cl:ensure-generic-function 'score-val :lambda-list '(m))
(cl:defmethod score-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:score-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:score instead.")
  (score m))

(cl:ensure-generic-function 'candidate_width_m-val :lambda-list '(m))
(cl:defmethod candidate_width_m-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:candidate_width_m-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:candidate_width_m instead.")
  (candidate_width_m m))

(cl:ensure-generic-function 'required_open_width_m-val :lambda-list '(m))
(cl:defmethod required_open_width_m-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:required_open_width_m-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:required_open_width_m instead.")
  (required_open_width_m m))

(cl:ensure-generic-function 'object_geometry-val :lambda-list '(m))
(cl:defmethod object_geometry-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:object_geometry-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:object_geometry instead.")
  (object_geometry m))

(cl:ensure-generic-function 'model_choice-val :lambda-list '(m))
(cl:defmethod model_choice-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:model_choice-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:model_choice instead.")
  (model_choice m))

(cl:ensure-generic-function 'plan_id-val :lambda-list '(m))
(cl:defmethod plan_id-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:plan_id-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:plan_id instead.")
  (plan_id m))

(cl:ensure-generic-function 'diagnostic-val :lambda-list '(m))
(cl:defmethod diagnostic-val ((m <Grasp6DPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:diagnostic-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:diagnostic instead.")
  (diagnostic m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Grasp6DPlan>) ostream)
  "Serializes a message object of type '<Grasp6DPlan>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poses))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poses))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'score))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'candidate_width_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'required_open_width_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_geometry) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model_choice))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model_choice))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'plan_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'plan_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'diagnostic))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'diagnostic))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Grasp6DPlan>) istream)
  "Deserializes a message object of type '<Grasp6DPlan>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poses) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poses)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'score) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'candidate_width_m) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'required_open_width_m) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_geometry) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model_choice) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model_choice) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'plan_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'plan_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'diagnostic) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'diagnostic) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Grasp6DPlan>)))
  "Returns string type for a message object of type '<Grasp6DPlan>"
  "alicia_flexible_grasp_supervisor/Grasp6DPlan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Grasp6DPlan)))
  "Returns string type for a message object of type 'Grasp6DPlan"
  "alicia_flexible_grasp_supervisor/Grasp6DPlan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Grasp6DPlan>)))
  "Returns md5sum for a message object of type '<Grasp6DPlan>"
  "a90c722154d42b08b0800cd88d52ff9d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Grasp6DPlan)))
  "Returns md5sum for a message object of type 'Grasp6DPlan"
  "a90c722154d42b08b0800cd88d52ff9d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Grasp6DPlan>)))
  "Returns full string definition for message of type '<Grasp6DPlan>"
  (cl:format cl:nil "std_msgs/Header header~%bool valid~%geometry_msgs/Pose[] poses~%float32 score~%float32 candidate_width_m~%float32 required_open_width_m~%alicia_flexible_grasp_supervisor/ObjectGeometry object_geometry~%string model_choice~%string plan_id~%string diagnostic~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: alicia_flexible_grasp_supervisor/ObjectGeometry~%std_msgs/Header header~%bool valid~%string label~%string source_mode~%geometry_msgs/Pose pose_base~%geometry_msgs/Vector3 size_xyz_m~%geometry_msgs/Vector3 support_normal_base~%float32 support_offset_m~%uint32 valid_depth_points~%float32 valid_depth_ratio~%float32 depth_mad_m~%uint32 fused_frames~%float32 support_inlier_ratio~%uint32 object_point_count~%string failure_reason~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Grasp6DPlan)))
  "Returns full string definition for message of type 'Grasp6DPlan"
  (cl:format cl:nil "std_msgs/Header header~%bool valid~%geometry_msgs/Pose[] poses~%float32 score~%float32 candidate_width_m~%float32 required_open_width_m~%alicia_flexible_grasp_supervisor/ObjectGeometry object_geometry~%string model_choice~%string plan_id~%string diagnostic~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: alicia_flexible_grasp_supervisor/ObjectGeometry~%std_msgs/Header header~%bool valid~%string label~%string source_mode~%geometry_msgs/Pose pose_base~%geometry_msgs/Vector3 size_xyz_m~%geometry_msgs/Vector3 support_normal_base~%float32 support_offset_m~%uint32 valid_depth_points~%float32 valid_depth_ratio~%float32 depth_mad_m~%uint32 fused_frames~%float32 support_inlier_ratio~%uint32 object_point_count~%string failure_reason~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Grasp6DPlan>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poses) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_geometry))
     4 (cl:length (cl:slot-value msg 'model_choice))
     4 (cl:length (cl:slot-value msg 'plan_id))
     4 (cl:length (cl:slot-value msg 'diagnostic))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Grasp6DPlan>))
  "Converts a ROS message object to a list"
  (cl:list 'Grasp6DPlan
    (cl:cons ':header (header msg))
    (cl:cons ':valid (valid msg))
    (cl:cons ':poses (poses msg))
    (cl:cons ':score (score msg))
    (cl:cons ':candidate_width_m (candidate_width_m msg))
    (cl:cons ':required_open_width_m (required_open_width_m msg))
    (cl:cons ':object_geometry (object_geometry msg))
    (cl:cons ':model_choice (model_choice msg))
    (cl:cons ':plan_id (plan_id msg))
    (cl:cons ':diagnostic (diagnostic msg))
))
