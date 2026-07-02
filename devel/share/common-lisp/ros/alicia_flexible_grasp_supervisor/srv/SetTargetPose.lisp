; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude SetTargetPose-request.msg.html

(cl:defclass <SetTargetPose-request> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (execute
    :reader execute
    :initarg :execute
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetTargetPose-request (<SetTargetPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTargetPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTargetPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<SetTargetPose-request> is deprecated: use alicia_flexible_grasp_supervisor-srv:SetTargetPose-request instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <SetTargetPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:target-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:target instead.")
  (target m))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <SetTargetPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:execute-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTargetPose-request>) ostream)
  "Serializes a message object of type '<SetTargetPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTargetPose-request>) istream)
  "Deserializes a message object of type '<SetTargetPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
    (cl:setf (cl:slot-value msg 'execute) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTargetPose-request>)))
  "Returns string type for a service object of type '<SetTargetPose-request>"
  "alicia_flexible_grasp_supervisor/SetTargetPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTargetPose-request)))
  "Returns string type for a service object of type 'SetTargetPose-request"
  "alicia_flexible_grasp_supervisor/SetTargetPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTargetPose-request>)))
  "Returns md5sum for a message object of type '<SetTargetPose-request>"
  "2cf3c52fcc5b5b6cce00c68f65c0b916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTargetPose-request)))
  "Returns md5sum for a message object of type 'SetTargetPose-request"
  "2cf3c52fcc5b5b6cce00c68f65c0b916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTargetPose-request>)))
  "Returns full string definition for message of type '<SetTargetPose-request>"
  (cl:format cl:nil "geometry_msgs/PoseStamped target~%bool execute~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTargetPose-request)))
  "Returns full string definition for message of type 'SetTargetPose-request"
  (cl:format cl:nil "geometry_msgs/PoseStamped target~%bool execute~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTargetPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTargetPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTargetPose-request
    (cl:cons ':target (target msg))
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude SetTargetPose-response.msg.html

(cl:defclass <SetTargetPose-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetTargetPose-response (<SetTargetPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetTargetPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetTargetPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<SetTargetPose-response> is deprecated: use alicia_flexible_grasp_supervisor-srv:SetTargetPose-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetTargetPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:success-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetTargetPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTargetPose-response>) ostream)
  "Serializes a message object of type '<SetTargetPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTargetPose-response>) istream)
  "Deserializes a message object of type '<SetTargetPose-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetTargetPose-response>)))
  "Returns string type for a service object of type '<SetTargetPose-response>"
  "alicia_flexible_grasp_supervisor/SetTargetPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTargetPose-response)))
  "Returns string type for a service object of type 'SetTargetPose-response"
  "alicia_flexible_grasp_supervisor/SetTargetPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetTargetPose-response>)))
  "Returns md5sum for a message object of type '<SetTargetPose-response>"
  "2cf3c52fcc5b5b6cce00c68f65c0b916")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTargetPose-response)))
  "Returns md5sum for a message object of type 'SetTargetPose-response"
  "2cf3c52fcc5b5b6cce00c68f65c0b916")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTargetPose-response>)))
  "Returns full string definition for message of type '<SetTargetPose-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTargetPose-response)))
  "Returns full string definition for message of type 'SetTargetPose-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTargetPose-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTargetPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTargetPose-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTargetPose)))
  'SetTargetPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTargetPose)))
  'SetTargetPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTargetPose)))
  "Returns string type for a service object of type '<SetTargetPose>"
  "alicia_flexible_grasp_supervisor/SetTargetPose")