; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-msg)


;//! \htmlinclude TactileState.msg.html

(cl:defclass <TactileState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (left
    :reader left
    :initarg :left
    :type alicia_flexible_grasp_supervisor-msg:TactileFrame
    :initform (cl:make-instance 'alicia_flexible_grasp_supervisor-msg:TactileFrame))
   (right
    :reader right
    :initarg :right
    :type alicia_flexible_grasp_supervisor-msg:TactileFrame
    :initform (cl:make-instance 'alicia_flexible_grasp_supervisor-msg:TactileFrame))
   (total_grip_force_mn
    :reader total_grip_force_mn
    :initarg :total_grip_force_mn
    :type cl:float
    :initform 0.0)
   (force_diff_mn
    :reader force_diff_mn
    :initarg :force_diff_mn
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
   (valid
    :reader valid
    :initarg :valid
    :type cl:boolean
    :initform cl:nil))
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

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:left-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:right-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'total_grip_force_mn-val :lambda-list '(m))
(cl:defmethod total_grip_force_mn-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:total_grip_force_mn-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:total_grip_force_mn instead.")
  (total_grip_force_mn m))

(cl:ensure-generic-function 'force_diff_mn-val :lambda-list '(m))
(cl:defmethod force_diff_mn-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:force_diff_mn-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:force_diff_mn instead.")
  (force_diff_mn m))

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

(cl:ensure-generic-function 'valid-val :lambda-list '(m))
(cl:defmethod valid-val ((m <TactileState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:valid-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:valid instead.")
  (valid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TactileState>) ostream)
  "Serializes a message object of type '<TactileState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'right) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'total_grip_force_mn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'force_diff_mn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_contact) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_contact) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'object_grasped) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'slip_detected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'valid) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TactileState>) istream)
  "Deserializes a message object of type '<TactileState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'right) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'total_grip_force_mn) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'force_diff_mn) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'left_contact) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_contact) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'object_grasped) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'slip_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'valid) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "5df6a8c43bd865ec0ec8d2f74fe1aa66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TactileState)))
  "Returns md5sum for a message object of type 'TactileState"
  "5df6a8c43bd865ec0ec8d2f74fe1aa66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TactileState>)))
  "Returns full string definition for message of type '<TactileState>"
  (cl:format cl:nil "std_msgs/Header header~%alicia_flexible_grasp_supervisor/TactileFrame left~%alicia_flexible_grasp_supervisor/TactileFrame right~%float32 total_grip_force_mn~%float32 force_diff_mn~%bool left_contact~%bool right_contact~%bool object_grasped~%bool slip_detected~%bool valid~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: alicia_flexible_grasp_supervisor/TactileFrame~%std_msgs/Header header~%string skin_name~%float32[] values~%uint32 rows~%uint32 cols~%float32 total_force_mn~%float32 max_force_mn~%uint32 max_index~%float32 center_x~%float32 center_y~%bool contact~%bool valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TactileState)))
  "Returns full string definition for message of type 'TactileState"
  (cl:format cl:nil "std_msgs/Header header~%alicia_flexible_grasp_supervisor/TactileFrame left~%alicia_flexible_grasp_supervisor/TactileFrame right~%float32 total_grip_force_mn~%float32 force_diff_mn~%bool left_contact~%bool right_contact~%bool object_grasped~%bool slip_detected~%bool valid~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: alicia_flexible_grasp_supervisor/TactileFrame~%std_msgs/Header header~%string skin_name~%float32[] values~%uint32 rows~%uint32 cols~%float32 total_force_mn~%float32 max_force_mn~%uint32 max_index~%float32 center_x~%float32 center_y~%bool contact~%bool valid~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TactileState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'right))
     4
     4
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TactileState>))
  "Converts a ROS message object to a list"
  (cl:list 'TactileState
    (cl:cons ':header (header msg))
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
    (cl:cons ':total_grip_force_mn (total_grip_force_mn msg))
    (cl:cons ':force_diff_mn (force_diff_mn msg))
    (cl:cons ':left_contact (left_contact msg))
    (cl:cons ':right_contact (right_contact msg))
    (cl:cons ':object_grasped (object_grasped msg))
    (cl:cons ':slip_detected (slip_detected msg))
    (cl:cons ':valid (valid msg))
))
