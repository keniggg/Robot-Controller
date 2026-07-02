; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude SetForceParams-request.msg.html

(cl:defclass <SetForceParams-request> (roslisp-msg-protocol:ros-message)
  ((contact_threshold_mn
    :reader contact_threshold_mn
    :initarg :contact_threshold_mn
    :type cl:float
    :initform 0.0)
   (target_force_mn
    :reader target_force_mn
    :initarg :target_force_mn
    :type cl:float
    :initform 0.0)
   (max_force_mn
    :reader max_force_mn
    :initarg :max_force_mn
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetForceParams-request (<SetForceParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetForceParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetForceParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<SetForceParams-request> is deprecated: use alicia_flexible_grasp_supervisor-srv:SetForceParams-request instead.")))

(cl:ensure-generic-function 'contact_threshold_mn-val :lambda-list '(m))
(cl:defmethod contact_threshold_mn-val ((m <SetForceParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:contact_threshold_mn-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:contact_threshold_mn instead.")
  (contact_threshold_mn m))

(cl:ensure-generic-function 'target_force_mn-val :lambda-list '(m))
(cl:defmethod target_force_mn-val ((m <SetForceParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:target_force_mn-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:target_force_mn instead.")
  (target_force_mn m))

(cl:ensure-generic-function 'max_force_mn-val :lambda-list '(m))
(cl:defmethod max_force_mn-val ((m <SetForceParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:max_force_mn-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:max_force_mn instead.")
  (max_force_mn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetForceParams-request>) ostream)
  "Serializes a message object of type '<SetForceParams-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'contact_threshold_mn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_force_mn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_force_mn))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetForceParams-request>) istream)
  "Deserializes a message object of type '<SetForceParams-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'contact_threshold_mn) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_force_mn) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_force_mn) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetForceParams-request>)))
  "Returns string type for a service object of type '<SetForceParams-request>"
  "alicia_flexible_grasp_supervisor/SetForceParamsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceParams-request)))
  "Returns string type for a service object of type 'SetForceParams-request"
  "alicia_flexible_grasp_supervisor/SetForceParamsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetForceParams-request>)))
  "Returns md5sum for a message object of type '<SetForceParams-request>"
  "b8d5117a765e68a9f9eef7b1389ac605")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetForceParams-request)))
  "Returns md5sum for a message object of type 'SetForceParams-request"
  "b8d5117a765e68a9f9eef7b1389ac605")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetForceParams-request>)))
  "Returns full string definition for message of type '<SetForceParams-request>"
  (cl:format cl:nil "float32 contact_threshold_mn~%float32 target_force_mn~%float32 max_force_mn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetForceParams-request)))
  "Returns full string definition for message of type 'SetForceParams-request"
  (cl:format cl:nil "float32 contact_threshold_mn~%float32 target_force_mn~%float32 max_force_mn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetForceParams-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetForceParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetForceParams-request
    (cl:cons ':contact_threshold_mn (contact_threshold_mn msg))
    (cl:cons ':target_force_mn (target_force_mn msg))
    (cl:cons ':max_force_mn (max_force_mn msg))
))
;//! \htmlinclude SetForceParams-response.msg.html

(cl:defclass <SetForceParams-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetForceParams-response (<SetForceParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetForceParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetForceParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<SetForceParams-response> is deprecated: use alicia_flexible_grasp_supervisor-srv:SetForceParams-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetForceParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:success-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetForceParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetForceParams-response>) ostream)
  "Serializes a message object of type '<SetForceParams-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetForceParams-response>) istream)
  "Deserializes a message object of type '<SetForceParams-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetForceParams-response>)))
  "Returns string type for a service object of type '<SetForceParams-response>"
  "alicia_flexible_grasp_supervisor/SetForceParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceParams-response)))
  "Returns string type for a service object of type 'SetForceParams-response"
  "alicia_flexible_grasp_supervisor/SetForceParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetForceParams-response>)))
  "Returns md5sum for a message object of type '<SetForceParams-response>"
  "b8d5117a765e68a9f9eef7b1389ac605")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetForceParams-response)))
  "Returns md5sum for a message object of type 'SetForceParams-response"
  "b8d5117a765e68a9f9eef7b1389ac605")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetForceParams-response>)))
  "Returns full string definition for message of type '<SetForceParams-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetForceParams-response)))
  "Returns full string definition for message of type 'SetForceParams-response"
  (cl:format cl:nil "bool success~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetForceParams-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetForceParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetForceParams-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetForceParams)))
  'SetForceParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetForceParams)))
  'SetForceParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceParams)))
  "Returns string type for a service object of type '<SetForceParams>"
  "alicia_flexible_grasp_supervisor/SetForceParams")