; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude StopGrasp-request.msg.html

(cl:defclass <StopGrasp-request> (roslisp-msg-protocol:ros-message)
  ((emergency
    :reader emergency
    :initarg :emergency
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StopGrasp-request (<StopGrasp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopGrasp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopGrasp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<StopGrasp-request> is deprecated: use alicia_flexible_grasp_supervisor-srv:StopGrasp-request instead.")))

(cl:ensure-generic-function 'emergency-val :lambda-list '(m))
(cl:defmethod emergency-val ((m <StopGrasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:emergency-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:emergency instead.")
  (emergency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopGrasp-request>) ostream)
  "Serializes a message object of type '<StopGrasp-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopGrasp-request>) istream)
  "Deserializes a message object of type '<StopGrasp-request>"
    (cl:setf (cl:slot-value msg 'emergency) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopGrasp-request>)))
  "Returns string type for a service object of type '<StopGrasp-request>"
  "alicia_flexible_grasp_supervisor/StopGraspRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopGrasp-request)))
  "Returns string type for a service object of type 'StopGrasp-request"
  "alicia_flexible_grasp_supervisor/StopGraspRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopGrasp-request>)))
  "Returns md5sum for a message object of type '<StopGrasp-request>"
  "5e6527b0f131fa5341cb29f70ba6b894")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopGrasp-request)))
  "Returns md5sum for a message object of type 'StopGrasp-request"
  "5e6527b0f131fa5341cb29f70ba6b894")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopGrasp-request>)))
  "Returns full string definition for message of type '<StopGrasp-request>"
  (cl:format cl:nil "bool emergency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopGrasp-request)))
  "Returns full string definition for message of type 'StopGrasp-request"
  (cl:format cl:nil "bool emergency~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopGrasp-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopGrasp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StopGrasp-request
    (cl:cons ':emergency (emergency msg))
))
;//! \htmlinclude StopGrasp-response.msg.html

(cl:defclass <StopGrasp-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass StopGrasp-response (<StopGrasp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StopGrasp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StopGrasp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<StopGrasp-response> is deprecated: use alicia_flexible_grasp_supervisor-srv:StopGrasp-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StopGrasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:success-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <StopGrasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StopGrasp-response>) ostream)
  "Serializes a message object of type '<StopGrasp-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StopGrasp-response>) istream)
  "Deserializes a message object of type '<StopGrasp-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StopGrasp-response>)))
  "Returns string type for a service object of type '<StopGrasp-response>"
  "alicia_flexible_grasp_supervisor/StopGraspResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopGrasp-response)))
  "Returns string type for a service object of type 'StopGrasp-response"
  "alicia_flexible_grasp_supervisor/StopGraspResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StopGrasp-response>)))
  "Returns md5sum for a message object of type '<StopGrasp-response>"
  "5e6527b0f131fa5341cb29f70ba6b894")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StopGrasp-response)))
  "Returns md5sum for a message object of type 'StopGrasp-response"
  "5e6527b0f131fa5341cb29f70ba6b894")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StopGrasp-response>)))
  "Returns full string definition for message of type '<StopGrasp-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StopGrasp-response)))
  "Returns full string definition for message of type 'StopGrasp-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StopGrasp-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StopGrasp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StopGrasp-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StopGrasp)))
  'StopGrasp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StopGrasp)))
  'StopGrasp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StopGrasp)))
  "Returns string type for a service object of type '<StopGrasp>"
  "alicia_flexible_grasp_supervisor/StopGrasp")