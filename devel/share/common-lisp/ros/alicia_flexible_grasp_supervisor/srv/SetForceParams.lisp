; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude SetForceParams-request.msg.html

(cl:defclass <SetForceParams-request> (roslisp-msg-protocol:ros-message)
  ((contact_threshold
    :reader contact_threshold
    :initarg :contact_threshold
    :type cl:float
    :initform 0.0)
   (target_force
    :reader target_force
    :initarg :target_force
    :type cl:float
    :initform 0.0)
   (max_force
    :reader max_force
    :initarg :max_force
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetForceParams-request (<SetForceParams-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetForceParams-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetForceParams-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<SetForceParams-request> is deprecated: use alicia_flexible_grasp_supervisor-srv:SetForceParams-request instead.")))

(cl:ensure-generic-function 'contact_threshold-val :lambda-list '(m))
(cl:defmethod contact_threshold-val ((m <SetForceParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:contact_threshold-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:contact_threshold instead.")
  (contact_threshold m))

(cl:ensure-generic-function 'target_force-val :lambda-list '(m))
(cl:defmethod target_force-val ((m <SetForceParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:target_force-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:target_force instead.")
  (target_force m))

(cl:ensure-generic-function 'max_force-val :lambda-list '(m))
(cl:defmethod max_force-val ((m <SetForceParams-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:max_force-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:max_force instead.")
  (max_force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetForceParams-request>) ostream)
  "Serializes a message object of type '<SetForceParams-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'contact_threshold))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'target_force))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_force))))
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
    (cl:setf (cl:slot-value msg 'contact_threshold) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_force) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_force) (roslisp-utils:decode-single-float-bits bits)))
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
  "5ef41667b487a681d2b7b91f8a1e0b10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetForceParams-request)))
  "Returns md5sum for a message object of type 'SetForceParams-request"
  "5ef41667b487a681d2b7b91f8a1e0b10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetForceParams-request>)))
  "Returns full string definition for message of type '<SetForceParams-request>"
  (cl:format cl:nil "float32 contact_threshold~%float32 target_force~%float32 max_force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetForceParams-request)))
  "Returns full string definition for message of type 'SetForceParams-request"
  (cl:format cl:nil "float32 contact_threshold~%float32 target_force~%float32 max_force~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetForceParams-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetForceParams-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetForceParams-request
    (cl:cons ':contact_threshold (contact_threshold msg))
    (cl:cons ':target_force (target_force msg))
    (cl:cons ':max_force (max_force msg))
))
;//! \htmlinclude SetForceParams-response.msg.html

(cl:defclass <SetForceParams-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetForceParams-response (<SetForceParams-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetForceParams-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetForceParams-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<SetForceParams-response> is deprecated: use alicia_flexible_grasp_supervisor-srv:SetForceParams-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <SetForceParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:ok-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:ok instead.")
  (ok m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetForceParams-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetForceParams-response>) ostream)
  "Serializes a message object of type '<SetForceParams-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetForceParams-response>) istream)
  "Deserializes a message object of type '<SetForceParams-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetForceParams-response>)))
  "Returns string type for a service object of type '<SetForceParams-response>"
  "alicia_flexible_grasp_supervisor/SetForceParamsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceParams-response)))
  "Returns string type for a service object of type 'SetForceParams-response"
  "alicia_flexible_grasp_supervisor/SetForceParamsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetForceParams-response>)))
  "Returns md5sum for a message object of type '<SetForceParams-response>"
  "5ef41667b487a681d2b7b91f8a1e0b10")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetForceParams-response)))
  "Returns md5sum for a message object of type 'SetForceParams-response"
  "5ef41667b487a681d2b7b91f8a1e0b10")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetForceParams-response>)))
  "Returns full string definition for message of type '<SetForceParams-response>"
  (cl:format cl:nil "bool ok~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetForceParams-response)))
  "Returns full string definition for message of type 'SetForceParams-response"
  (cl:format cl:nil "bool ok~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetForceParams-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetForceParams-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetForceParams-response
    (cl:cons ':ok (ok msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetForceParams)))
  'SetForceParams-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetForceParams)))
  'SetForceParams-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetForceParams)))
  "Returns string type for a service object of type '<SetForceParams>"
  "alicia_flexible_grasp_supervisor/SetForceParams")