; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-msg)


;//! \htmlinclude TactileState.msg.html

(cl:defclass <TactileState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (skin1
    :reader skin1
    :initarg :skin1
    :type alicia_flexible_grasp_supervisor-msg:TactileFrame
    :initform (cl:make-instance 'alicia_flexible_grasp_supervisor-msg:TactileFrame))
   (skin2
    :reader skin2
    :initarg :skin2
    :type alicia_flexible_grasp_supervisor-msg:TactileFrame
    :initform (cl:make-instance 'alicia_flexible_grasp_supervisor-msg:TactileFrame))
   (total_grip_force
    :reader total_grip_force
    :initarg :total_grip_force
    :type cl:float
    :initform 0.0)
   (force_diff
    :reader force_diff
    :initarg :force_diff
    :type cl:float
    :initform 0.0)
   (left_contact
    :reader left_contact
    :initarg :left_contact
    :type cl:boolean
    :initform cl:nil)
   (right_contact
    :reader right_contact
    :initarg :right_contact
    :type cl:boolean
    :initform cl:nil)
   (object_grasped
    :reader object_grasped
    :initarg :object_grasped
    :type cl:boolean
    :initform cl:nil)
   (slip_detected
    :reader slip_detected
    :initarg :slip_detected
    :type cl:boolean
    :initform cl:nil)
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass TactileState (<TactileState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TactileState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TactileState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-msg:<TactileState> is deprecated: use alicia_flexible_grasp_supervisor-msg:TactileState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:header-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'skin1-val :lambda-list '(m))
(cl:defmethod skin1-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:skin1-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:skin1 instead.")
  (skin1 m))

(cl:ensure-generic-function 'skin2-val :lambda-list '(m))
(cl:defmethod skin2-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:skin2-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:skin2 instead.")
  (skin2 m))

(cl:ensure-generic-function 'total_grip_force-val :lambda-list '(m))
(cl:defmethod total_grip_force-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:total_grip_force-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:total_grip_force instead.")
  (total_grip_force m))

(cl:ensure-generic-function 'force_diff-val :lambda-list '(m))
(cl:defmethod force_diff-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:force_diff-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:force_diff instead.")
  (force_diff m))

(cl:ensure-generic-function 'left_contact-val :lambda-list '(m))
(cl:defmethod left_contact-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:left_contact-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:left_contact instead.")
  (left_contact m))

(cl:ensure-generic-function 'right_contact-val :lambda-list '(m))
(cl:defmethod right_contact-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:right_contact-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:right_contact instead.")
  (right_contact m))

(cl:ensure-generic-function 'object_grasped-val :lambda-list '(m))
(cl:defmethod object_grasped-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:object_grasped-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:object_grasped instead.")
  (object_grasped m))

(cl:ensure-generic-function 'slip_detected-val :lambda-list '(m))
(cl:defmethod slip_detected-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:slip_detected-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:slip_detected instead.")
  (slip_detected m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:status-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TactileState>) ostream)
  "Serializes a message object of type '<TactileState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'skin1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'skin2) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_grip_force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force_diff))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_contact) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_contact) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'object_grasped) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'slip_detected) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TactileState>) istream)
  "Deserializes a message object of type '<TactileState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'skin1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'skin2) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_grip_force) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force_diff) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'left_contact) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_contact) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'object_grasped) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'slip_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TactileState>)))
  "Returns string type for a message object of type '<TactileState>"
  "alicia_flexible_grasp_supervisor/TactileState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TactileState)))
  "Returns string type for a message object of type 'TactileState"
  "alicia_flexible_grasp_supervisor/TactileState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TactileState>)))
  "Returns md5sum for a message object of type '<TactileState>"
  "9b76704935dbe875f8fc5a941f64277a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TactileState)))
  "Returns md5sum for a message object of type 'TactileState"
  "9b76704935dbe875f8fc5a941f64277a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TactileState>)))
  "Returns full string definition for message of type '<TactileState>"
  (cl:format cl:nil "std_msgs/Header header~%alicia_flexible_grasp_supervisor/TactileFrame skin1~%alicia_flexible_grasp_supervisor/TactileFrame skin2~%float32 total_grip_force~%float32 force_diff~%bool left_contact~%bool right_contact~%bool object_grasped~%bool slip_detected~%string status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: alicia_flexible_grasp_supervisor/TactileFrame~%std_msgs/Header header~%uint8 skin_id~%uint16 rows~%uint16 cols~%float32[] values~%float32 total_force~%float32 max_force~%uint16 max_index~%float32 center_x~%float32 center_y~%bool contact~%bool valid~%string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TactileState)))
  "Returns full string definition for message of type 'TactileState"
  (cl:format cl:nil "std_msgs/Header header~%alicia_flexible_grasp_supervisor/TactileFrame skin1~%alicia_flexible_grasp_supervisor/TactileFrame skin2~%float32 total_grip_force~%float32 force_diff~%bool left_contact~%bool right_contact~%bool object_grasped~%bool slip_detected~%string status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: alicia_flexible_grasp_supervisor/TactileFrame~%std_msgs/Header header~%uint8 skin_id~%uint16 rows~%uint16 cols~%float32[] values~%float32 total_force~%float32 max_force~%uint16 max_index~%float32 center_x~%float32 center_y~%bool contact~%bool valid~%string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TactileState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'skin1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'skin2))
     4
     4
     1
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TactileState>))
  "Converts a ROS message object to a list"
  (cl:list 'TactileState
    (cl:cons ':header (header msg))
    (cl:cons ':skin1 (skin1 msg))
    (cl:cons ':skin2 (skin2 msg))
    (cl:cons ':total_grip_force (total_grip_force msg))
    (cl:cons ':force_diff (force_diff msg))
    (cl:cons ':left_contact (left_contact msg))
    (cl:cons ':right_contact (right_contact msg))
    (cl:cons ':object_grasped (object_grasped msg))
    (cl:cons ':slip_detected (slip_detected msg))
    (cl:cons ':status (status msg))
))
