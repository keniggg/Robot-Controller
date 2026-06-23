; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude StartGrasp-request.msg.html

(cl:defclass <StartGrasp-request> (roslisp-msg-protocol:ros-message)
  ((use_latest_object
    :reader use_latest_object
    :initarg :use_latest_object
    :type cl:boolean
    :initform cl:nil)
   (object_label
    :reader object_label
    :initarg :object_label
    :type cl:string
    :initform ""))
)

(cl:defclass StartGrasp-request (<StartGrasp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartGrasp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartGrasp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<StartGrasp-request> is deprecated: use alicia_flexible_grasp_supervisor-srv:StartGrasp-request instead.")))

(cl:ensure-generic-function 'use_latest_object-val :lambda-list '(m))
(cl:defmethod use_latest_object-val ((m <StartGrasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:use_latest_object-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:use_latest_object instead.")
  (use_latest_object m))

(cl:ensure-generic-function 'object_label-val :lambda-list '(m))
(cl:defmethod object_label-val ((m <StartGrasp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:object_label-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:object_label instead.")
  (object_label m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartGrasp-request>) ostream)
  "Serializes a message object of type '<StartGrasp-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'use_latest_object) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'object_label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'object_label))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartGrasp-request>) istream)
  "Deserializes a message object of type '<StartGrasp-request>"
    (cl:setf (cl:slot-value msg 'use_latest_object) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'object_label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'object_label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
  "444b03b3725d049d9ce9ac0a3dfa2c54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartGrasp-request)))
  "Returns md5sum for a message object of type 'StartGrasp-request"
  "444b03b3725d049d9ce9ac0a3dfa2c54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartGrasp-request>)))
  "Returns full string definition for message of type '<StartGrasp-request>"
  (cl:format cl:nil "bool use_latest_object~%string object_label~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartGrasp-request)))
  "Returns full string definition for message of type 'StartGrasp-request"
  (cl:format cl:nil "bool use_latest_object~%string object_label~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartGrasp-request>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'object_label))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartGrasp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartGrasp-request
    (cl:cons ':use_latest_object (use_latest_object msg))
    (cl:cons ':object_label (object_label msg))
))
;//! \htmlinclude StartGrasp-response.msg.html

(cl:defclass <StartGrasp-response> (roslisp-msg-protocol:ros-message)
  ((accepted
    :reader accepted
    :initarg :accepted
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

(cl:ensure-generic-function 'accepted-val :lambda-list '(m))
(cl:defmethod accepted-val ((m <StartGrasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:accepted-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:accepted instead.")
  (accepted m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <StartGrasp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartGrasp-response>) ostream)
  "Serializes a message object of type '<StartGrasp-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'accepted) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartGrasp-response>) istream)
  "Deserializes a message object of type '<StartGrasp-response>"
    (cl:setf (cl:slot-value msg 'accepted) (cl:not (cl:zerop (cl:read-byte istream))))
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
  "444b03b3725d049d9ce9ac0a3dfa2c54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartGrasp-response)))
  "Returns md5sum for a message object of type 'StartGrasp-response"
  "444b03b3725d049d9ce9ac0a3dfa2c54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartGrasp-response>)))
  "Returns full string definition for message of type '<StartGrasp-response>"
  (cl:format cl:nil "bool accepted~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartGrasp-response)))
  "Returns full string definition for message of type 'StartGrasp-response"
  (cl:format cl:nil "bool accepted~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartGrasp-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartGrasp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartGrasp-response
    (cl:cons ':accepted (accepted msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartGrasp)))
  'StartGrasp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartGrasp)))
  'StartGrasp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartGrasp)))
  "Returns string type for a service object of type '<StartGrasp>"
  "alicia_flexible_grasp_supervisor/StartGrasp")