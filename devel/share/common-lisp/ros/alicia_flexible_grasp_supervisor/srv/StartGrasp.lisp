; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude StartGrasp-request.msg.html

(cl:defclass <StartGrasp-request> (roslisp-msg-protocol:ros-message)
  ((execute
    :reader execute
    :initarg :execute
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass StartGrasp-request (<StartGrasp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartGrasp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartGrasp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<StartGrasp-request> is deprecated: use alicia_flexible_grasp_supervisor-srv:StartGrasp-request instead.")))

(cl:ensure-generic-function 'execute-val :lambda-list '(m))
(cl:defmethod execute-val ((m <StartGrasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:execute-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:execute instead.")
  (execute m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartGrasp-request>) ostream)
  "Serializes a message object of type '<StartGrasp-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartGrasp-request>) istream)
  "Deserializes a message object of type '<StartGrasp-request>"
    (cl:setf (cl:slot-value msg 'execute) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartGrasp-request>)))
  "Returns string type for a service object of type '<StartGrasp-request>"
  "alicia_flexible_grasp_supervisor/StartGraspRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartGrasp-request)))
  "Returns string type for a service object of type 'StartGrasp-request"
  "alicia_flexible_grasp_supervisor/StartGraspRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartGrasp-request>)))
  "Returns md5sum for a message object of type '<StartGrasp-request>"
  "c1e3198b68b143183a952c85cd9f744a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartGrasp-request)))
  "Returns md5sum for a message object of type 'StartGrasp-request"
  "c1e3198b68b143183a952c85cd9f744a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartGrasp-request>)))
  "Returns full string definition for message of type '<StartGrasp-request>"
  (cl:format cl:nil "bool execute~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartGrasp-request)))
  "Returns full string definition for message of type 'StartGrasp-request"
  (cl:format cl:nil "bool execute~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartGrasp-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartGrasp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartGrasp-request
    (cl:cons ':execute (execute msg))
))
;//! \htmlinclude StartGrasp-response.msg.html

(cl:defclass <StartGrasp-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass StartGrasp-response (<StartGrasp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartGrasp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartGrasp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<StartGrasp-response> is deprecated: use alicia_flexible_grasp_supervisor-srv:StartGrasp-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <StartGrasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:success-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <StartGrasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartGrasp-response>) ostream)
  "Serializes a message object of type '<StartGrasp-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartGrasp-response>) istream)
  "Deserializes a message object of type '<StartGrasp-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartGrasp-response>)))
  "Returns string type for a service object of type '<StartGrasp-response>"
  "alicia_flexible_grasp_supervisor/StartGraspResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartGrasp-response)))
  "Returns string type for a service object of type 'StartGrasp-response"
  "alicia_flexible_grasp_supervisor/StartGraspResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartGrasp-response>)))
  "Returns md5sum for a message object of type '<StartGrasp-response>"
  "c1e3198b68b143183a952c85cd9f744a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartGrasp-response)))
  "Returns md5sum for a message object of type 'StartGrasp-response"
  "c1e3198b68b143183a952c85cd9f744a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartGrasp-response>)))
  "Returns full string definition for message of type '<StartGrasp-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartGrasp-response)))
  "Returns full string definition for message of type 'StartGrasp-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartGrasp-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartGrasp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartGrasp-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartGrasp)))
  'StartGrasp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartGrasp)))
  'StartGrasp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartGrasp)))
  "Returns string type for a service object of type '<StartGrasp>"
  "alicia_flexible_grasp_supervisor/StartGrasp")