; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude SetTargetPose-request.msg.html

(cl:defclass <SetTargetPose-request> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
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

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <SetTargetPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:pose-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:pose instead.")
  (pose m))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <SetTargetPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:execute-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTargetPose-request>) ostream)
  "Serializes a message object of type '<SetTargetPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTargetPose-request>) istream)
  "Deserializes a message object of type '<SetTargetPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
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
  "00e64715c2cedd5add3400704911c3af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTargetPose-request)))
  "Returns md5sum for a message object of type 'SetTargetPose-request"
  "00e64715c2cedd5add3400704911c3af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTargetPose-request>)))
  "Returns full string definition for message of type '<SetTargetPose-request>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTargetPose-request)))
  "Returns full string definition for message of type 'SetTargetPose-request"
  (cl:format cl:nil "geometry_msgs/Pose pose~%bool execute~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTargetPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTargetPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTargetPose-request
    (cl:cons ':pose (pose msg))
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude SetTargetPose-response.msg.html

(cl:defclass <SetTargetPose-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
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

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetTargetPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:ok-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetTargetPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetTargetPose-response>) ostream)
  "Serializes a message object of type '<SetTargetPose-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetTargetPose-response>) istream)
  "Deserializes a message object of type '<SetTargetPose-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "00e64715c2cedd5add3400704911c3af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetTargetPose-response)))
  "Returns md5sum for a message object of type 'SetTargetPose-response"
  "00e64715c2cedd5add3400704911c3af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetTargetPose-response>)))
  "Returns full string definition for message of type '<SetTargetPose-response>"
  (cl:format cl:nil "bool ok~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetTargetPose-response)))
  "Returns full string definition for message of type 'SetTargetPose-response"
  (cl:format cl:nil "bool ok~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetTargetPose-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetTargetPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetTargetPose-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetTargetPose)))
  'SetTargetPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetTargetPose)))
  'SetTargetPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetTargetPose)))
  "Returns string type for a service object of type '<SetTargetPose>"
  "alicia_flexible_grasp_supervisor/SetTargetPose")