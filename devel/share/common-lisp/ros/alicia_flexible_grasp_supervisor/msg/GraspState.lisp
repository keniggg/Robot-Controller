; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-msg)


;//! \htmlinclude GraspState.msg.html

(cl:defclass <GraspState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (state_name
    :reader state_name
    :initarg :state_name
    :type cl:string
    :initform "")
   (running
    :reader running
    :initarg :running
    :type cl:boolean
    :initform cl:nil)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (current_force
    :reader current_force
    :initarg :current_force
    :type cl:float
    :initform 0.0)
   (target_force
    :reader target_force
    :initarg :target_force
    :type cl:float
    :initform 0.0)
   (object_pose_base
    :reader object_pose_base
    :initarg :object_pose_base
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (target_pose
    :reader target_pose
    :initarg :target_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass GraspState (<GraspState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-msg:<GraspState> is deprecated: use alicia_flexible_grasp_supervisor-msg:GraspState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:header-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:state-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'state_name-val :lambda-list '(m))
(cl:defmethod state_name-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:state_name-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:state_name instead.")
  (state_name m))

(cl:ensure-generic-function 'running-val :lambda-list '(m))
(cl:defmethod running-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:running-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:running instead.")
  (running m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:success-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:message instead.")
  (message m))

(cl:ensure-generic-function 'current_force-val :lambda-list '(m))
(cl:defmethod current_force-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:current_force-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:current_force instead.")
  (current_force m))

(cl:ensure-generic-function 'target_force-val :lambda-list '(m))
(cl:defmethod target_force-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:target_force-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:target_force instead.")
  (target_force m))

(cl:ensure-generic-function 'object_pose_base-val :lambda-list '(m))
(cl:defmethod object_pose_base-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:object_pose_base-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:object_pose_base instead.")
  (object_pose_base m))

(cl:ensure-generic-function 'target_pose-val :lambda-list '(m))
(cl:defmethod target_pose-val ((m <GraspState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:target_pose-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:target_pose instead.")
  (target_pose m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<GraspState>)))
    "Constants for message type '<GraspState>"
  '((:IDLE . 0)
    (:SEARCH_OBJECT . 1)
    (:ESTIMATE_POSE . 2)
    (:PLAN_PREGRASP . 3)
    (:MOVE_PREGRASP . 4)
    (:APPROACH_OBJECT . 5)
    (:COMPLIANT_CLOSE . 6)
    (:GRASP_VERIFY . 7)
    (:LIFT_OBJECT . 8)
    (:PLACE_OBJECT . 9)
    (:RELEASE_OBJECT . 10)
    (:SUCCESS . 11)
    (:FAILED . 12)
    (:EMERGENCY_STOP . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'GraspState)))
    "Constants for message type 'GraspState"
  '((:IDLE . 0)
    (:SEARCH_OBJECT . 1)
    (:ESTIMATE_POSE . 2)
    (:PLAN_PREGRASP . 3)
    (:MOVE_PREGRASP . 4)
    (:APPROACH_OBJECT . 5)
    (:COMPLIANT_CLOSE . 6)
    (:GRASP_VERIFY . 7)
    (:LIFT_OBJECT . 8)
    (:PLACE_OBJECT . 9)
    (:RELEASE_OBJECT . 10)
    (:SUCCESS . 11)
    (:FAILED . 12)
    (:EMERGENCY_STOP . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspState>) ostream)
  "Serializes a message object of type '<GraspState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state_name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'running) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'object_pose_base) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspState>) istream)
  "Deserializes a message object of type '<GraspState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'running) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_force) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_force) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'object_pose_base) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspState>)))
  "Returns string type for a message object of type '<GraspState>"
  "alicia_flexible_grasp_supervisor/GraspState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspState)))
  "Returns string type for a message object of type 'GraspState"
  "alicia_flexible_grasp_supervisor/GraspState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspState>)))
  "Returns md5sum for a message object of type '<GraspState>"
  "75215c45032076e51dbd82164d6951cb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspState)))
  "Returns md5sum for a message object of type 'GraspState"
  "75215c45032076e51dbd82164d6951cb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspState>)))
  "Returns full string definition for message of type '<GraspState>"
  (cl:format cl:nil "std_msgs/Header header~%uint8 IDLE=0~%uint8 SEARCH_OBJECT=1~%uint8 ESTIMATE_POSE=2~%uint8 PLAN_PREGRASP=3~%uint8 MOVE_PREGRASP=4~%uint8 APPROACH_OBJECT=5~%uint8 COMPLIANT_CLOSE=6~%uint8 GRASP_VERIFY=7~%uint8 LIFT_OBJECT=8~%uint8 PLACE_OBJECT=9~%uint8 RELEASE_OBJECT=10~%uint8 SUCCESS=11~%uint8 FAILED=12~%uint8 EMERGENCY_STOP=13~%uint8 state~%string state_name~%bool running~%bool success~%string message~%float32 current_force~%float32 target_force~%geometry_msgs/Pose object_pose_base~%geometry_msgs/Pose target_pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspState)))
  "Returns full string definition for message of type 'GraspState"
  (cl:format cl:nil "std_msgs/Header header~%uint8 IDLE=0~%uint8 SEARCH_OBJECT=1~%uint8 ESTIMATE_POSE=2~%uint8 PLAN_PREGRASP=3~%uint8 MOVE_PREGRASP=4~%uint8 APPROACH_OBJECT=5~%uint8 COMPLIANT_CLOSE=6~%uint8 GRASP_VERIFY=7~%uint8 LIFT_OBJECT=8~%uint8 PLACE_OBJECT=9~%uint8 RELEASE_OBJECT=10~%uint8 SUCCESS=11~%uint8 FAILED=12~%uint8 EMERGENCY_STOP=13~%uint8 state~%string state_name~%bool running~%bool success~%string message~%float32 current_force~%float32 target_force~%geometry_msgs/Pose object_pose_base~%geometry_msgs/Pose target_pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'state_name))
     1
     1
     4 (cl:length (cl:slot-value msg 'message))
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'object_pose_base))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspState>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspState
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':state_name (state_name msg))
    (cl:cons ':running (running msg))
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':current_force (current_force msg))
    (cl:cons ':target_force (target_force msg))
    (cl:cons ':object_pose_base (object_pose_base msg))
    (cl:cons ':target_pose (target_pose msg))
))
