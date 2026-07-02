; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude TriggerZero-request.msg.html

(cl:defclass <TriggerZero-request> (roslisp-msg-protocol:ros-message)
  ((trigger
    :reader trigger
    :initarg :trigger
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass TriggerZero-request (<TriggerZero-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerZero-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerZero-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<TriggerZero-request> is deprecated: use alicia_flexible_grasp_supervisor-srv:TriggerZero-request instead.")))

(cl:ensure-generic-function 'trigger-val :lambda-list '(m))
(cl:defmethod trigger-val ((m <TriggerZero-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:trigger-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:trigger instead.")
  (trigger m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerZero-request>) ostream)
  "Serializes a message object of type '<TriggerZero-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'trigger) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerZero-request>) istream)
  "Deserializes a message object of type '<TriggerZero-request>"
    (cl:setf (cl:slot-value msg 'trigger) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerZero-request>)))
  "Returns string type for a service object of type '<TriggerZero-request>"
  "alicia_flexible_grasp_supervisor/TriggerZeroRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerZero-request)))
  "Returns string type for a service object of type 'TriggerZero-request"
  "alicia_flexible_grasp_supervisor/TriggerZeroRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerZero-request>)))
  "Returns md5sum for a message object of type '<TriggerZero-request>"
  "1531357144714babf83a2def976015d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerZero-request)))
  "Returns md5sum for a message object of type 'TriggerZero-request"
  "1531357144714babf83a2def976015d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerZero-request>)))
  "Returns full string definition for message of type '<TriggerZero-request>"
  (cl:format cl:nil "bool trigger~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerZero-request)))
  "Returns full string definition for message of type 'TriggerZero-request"
  (cl:format cl:nil "bool trigger~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerZero-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerZero-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerZero-request
    (cl:cons ':trigger (trigger msg))
))
;//! \htmlinclude TriggerZero-response.msg.html

(cl:defclass <TriggerZero-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass TriggerZero-response (<TriggerZero-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerZero-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerZero-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<TriggerZero-response> is deprecated: use alicia_flexible_grasp_supervisor-srv:TriggerZero-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TriggerZero-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:success-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <TriggerZero-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerZero-response>) ostream)
  "Serializes a message object of type '<TriggerZero-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerZero-response>) istream)
  "Deserializes a message object of type '<TriggerZero-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerZero-response>)))
  "Returns string type for a service object of type '<TriggerZero-response>"
  "alicia_flexible_grasp_supervisor/TriggerZeroResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerZero-response)))
  "Returns string type for a service object of type 'TriggerZero-response"
  "alicia_flexible_grasp_supervisor/TriggerZeroResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerZero-response>)))
  "Returns md5sum for a message object of type '<TriggerZero-response>"
  "1531357144714babf83a2def976015d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerZero-response)))
  "Returns md5sum for a message object of type 'TriggerZero-response"
  "1531357144714babf83a2def976015d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerZero-response>)))
  "Returns full string definition for message of type '<TriggerZero-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerZero-response)))
  "Returns full string definition for message of type 'TriggerZero-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerZero-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerZero-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerZero-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TriggerZero)))
  'TriggerZero-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TriggerZero)))
  'TriggerZero-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerZero)))
  "Returns string type for a service object of type '<TriggerZero>"
  "alicia_flexible_grasp_supervisor/TriggerZero")