; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-msg)


;//! \htmlinclude SafetyState.msg.html

(cl:defclass <SafetyState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil)
   (emergency_stop
    :reader emergency_stop
    :initarg :emergency_stop
    :type cl:boolean
    :initform cl:nil)
   (force_over_limit
    :reader force_over_limit
    :initarg :force_over_limit
    :type cl:boolean
    :initform cl:nil)
   (robot_timeout
    :reader robot_timeout
    :initarg :robot_timeout
    :type cl:boolean
    :initform cl:nil)
   (tactile_timeout
    :reader tactile_timeout
    :initarg :tactile_timeout
    :type cl:boolean
    :initform cl:nil)
   (camera_timeout
    :reader camera_timeout
    :initarg :camera_timeout
    :type cl:boolean
    :initform cl:nil)
   (planning_failed
    :reader planning_failed
    :initarg :planning_failed
    :type cl:boolean
    :initform cl:nil)
   (level
    :reader level
    :initarg :level
    :type cl:string
    :initform "")
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SafetyState (<SafetyState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SafetyState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SafetyState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-msg:<SafetyState> is deprecated: use alicia_flexible_grasp_supervisor-msg:SafetyState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:header-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:ok-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:ok instead.")
  (ok m))

(cl:ensure-generic-function 'emergency_stop-val :lambda-list '(m))
(cl:defmethod emergency_stop-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:emergency_stop-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:emergency_stop instead.")
  (emergency_stop m))

(cl:ensure-generic-function 'force_over_limit-val :lambda-list '(m))
(cl:defmethod force_over_limit-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:force_over_limit-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:force_over_limit instead.")
  (force_over_limit m))

(cl:ensure-generic-function 'robot_timeout-val :lambda-list '(m))
(cl:defmethod robot_timeout-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:robot_timeout-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:robot_timeout instead.")
  (robot_timeout m))

(cl:ensure-generic-function 'tactile_timeout-val :lambda-list '(m))
(cl:defmethod tactile_timeout-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:tactile_timeout-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:tactile_timeout instead.")
  (tactile_timeout m))

(cl:ensure-generic-function 'camera_timeout-val :lambda-list '(m))
(cl:defmethod camera_timeout-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:camera_timeout-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:camera_timeout instead.")
  (camera_timeout m))

(cl:ensure-generic-function 'planning_failed-val :lambda-list '(m))
(cl:defmethod planning_failed-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:planning_failed-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:planning_failed instead.")
  (planning_failed m))

(cl:ensure-generic-function 'level-val :lambda-list '(m))
(cl:defmethod level-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:level-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:level instead.")
  (level m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SafetyState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SafetyState>) ostream)
  "Serializes a message object of type '<SafetyState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency_stop) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'force_over_limit) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'robot_timeout) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tactile_timeout) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'camera_timeout) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'planning_failed) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'level))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'level))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SafetyState>) istream)
  "Deserializes a message object of type '<SafetyState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'emergency_stop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'force_over_limit) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'robot_timeout) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tactile_timeout) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'camera_timeout) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'planning_failed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'level) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'level) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SafetyState>)))
  "Returns string type for a message object of type '<SafetyState>"
  "alicia_flexible_grasp_supervisor/SafetyState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SafetyState)))
  "Returns string type for a message object of type 'SafetyState"
  "alicia_flexible_grasp_supervisor/SafetyState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SafetyState>)))
  "Returns md5sum for a message object of type '<SafetyState>"
  "d422c2c3e9933ef7c15274804ca0c2f7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SafetyState)))
  "Returns md5sum for a message object of type 'SafetyState"
  "d422c2c3e9933ef7c15274804ca0c2f7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SafetyState>)))
  "Returns full string definition for message of type '<SafetyState>"
  (cl:format cl:nil "std_msgs/Header header~%bool ok~%bool emergency_stop~%bool force_over_limit~%bool robot_timeout~%bool tactile_timeout~%bool camera_timeout~%bool planning_failed~%string level~%string message~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SafetyState)))
  "Returns full string definition for message of type 'SafetyState"
  (cl:format cl:nil "std_msgs/Header header~%bool ok~%bool emergency_stop~%bool force_over_limit~%bool robot_timeout~%bool tactile_timeout~%bool camera_timeout~%bool planning_failed~%string level~%string message~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SafetyState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     1
     1
     1
     4 (cl:length (cl:slot-value msg 'level))
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SafetyState>))
  "Converts a ROS message object to a list"
  (cl:list 'SafetyState
    (cl:cons ':header (header msg))
    (cl:cons ':ok (ok msg))
    (cl:cons ':emergency_stop (emergency_stop msg))
    (cl:cons ':force_over_limit (force_over_limit msg))
    (cl:cons ':robot_timeout (robot_timeout msg))
    (cl:cons ':tactile_timeout (tactile_timeout msg))
    (cl:cons ':camera_timeout (camera_timeout msg))
    (cl:cons ':planning_failed (planning_failed msg))
    (cl:cons ':level (level msg))
    (cl:cons ':message (message msg))
))
