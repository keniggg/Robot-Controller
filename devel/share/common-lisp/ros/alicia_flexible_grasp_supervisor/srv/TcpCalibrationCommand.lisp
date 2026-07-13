; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-srv)


;//! \htmlinclude TcpCalibrationCommand-request.msg.html

(cl:defclass <TcpCalibrationCommand-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass TcpCalibrationCommand-request (<TcpCalibrationCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TcpCalibrationCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TcpCalibrationCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<TcpCalibrationCommand-request> is deprecated: use alicia_flexible_grasp_supervisor-srv:TcpCalibrationCommand-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <TcpCalibrationCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:command-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TcpCalibrationCommand-request>) ostream)
  "Serializes a message object of type '<TcpCalibrationCommand-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TcpCalibrationCommand-request>) istream)
  "Deserializes a message object of type '<TcpCalibrationCommand-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TcpCalibrationCommand-request>)))
  "Returns string type for a service object of type '<TcpCalibrationCommand-request>"
  "alicia_flexible_grasp_supervisor/TcpCalibrationCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TcpCalibrationCommand-request)))
  "Returns string type for a service object of type 'TcpCalibrationCommand-request"
  "alicia_flexible_grasp_supervisor/TcpCalibrationCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TcpCalibrationCommand-request>)))
  "Returns md5sum for a message object of type '<TcpCalibrationCommand-request>"
  "58e49899f0eda63c395f3d3908079771")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TcpCalibrationCommand-request)))
  "Returns md5sum for a message object of type 'TcpCalibrationCommand-request"
  "58e49899f0eda63c395f3d3908079771")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TcpCalibrationCommand-request>)))
  "Returns full string definition for message of type '<TcpCalibrationCommand-request>"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TcpCalibrationCommand-request)))
  "Returns full string definition for message of type 'TcpCalibrationCommand-request"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TcpCalibrationCommand-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TcpCalibrationCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TcpCalibrationCommand-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude TcpCalibrationCommand-response.msg.html

(cl:defclass <TcpCalibrationCommand-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (sample_count
    :reader sample_count
    :initarg :sample_count
    :type cl:integer
    :initform 0)
   (tcp_translation
    :reader tcp_translation
    :initarg :tcp_translation
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (fixed_point
    :reader fixed_point
    :initarg :fixed_point
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (rms_error_m
    :reader rms_error_m
    :initarg :rms_error_m
    :type cl:float
    :initform 0.0)
   (max_error_m
    :reader max_error_m
    :initarg :max_error_m
    :type cl:float
    :initform 0.0)
   (orientation_span_deg
    :reader orientation_span_deg
    :initarg :orientation_span_deg
    :type cl:float
    :initform 0.0)
   (result_file
    :reader result_file
    :initarg :result_file
    :type cl:string
    :initform ""))
)

(cl:defclass TcpCalibrationCommand-response (<TcpCalibrationCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TcpCalibrationCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TcpCalibrationCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-srv:<TcpCalibrationCommand-response> is deprecated: use alicia_flexible_grasp_supervisor-srv:TcpCalibrationCommand-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:success-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:message-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'sample_count-val :lambda-list '(m))
(cl:defmethod sample_count-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:sample_count-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:sample_count instead.")
  (sample_count m))

(cl:ensure-generic-function 'tcp_translation-val :lambda-list '(m))
(cl:defmethod tcp_translation-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:tcp_translation-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:tcp_translation instead.")
  (tcp_translation m))

(cl:ensure-generic-function 'fixed_point-val :lambda-list '(m))
(cl:defmethod fixed_point-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:fixed_point-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:fixed_point instead.")
  (fixed_point m))

(cl:ensure-generic-function 'rms_error_m-val :lambda-list '(m))
(cl:defmethod rms_error_m-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:rms_error_m-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:rms_error_m instead.")
  (rms_error_m m))

(cl:ensure-generic-function 'max_error_m-val :lambda-list '(m))
(cl:defmethod max_error_m-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:max_error_m-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:max_error_m instead.")
  (max_error_m m))

(cl:ensure-generic-function 'orientation_span_deg-val :lambda-list '(m))
(cl:defmethod orientation_span_deg-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:orientation_span_deg-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:orientation_span_deg instead.")
  (orientation_span_deg m))

(cl:ensure-generic-function 'result_file-val :lambda-list '(m))
(cl:defmethod result_file-val ((m <TcpCalibrationCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-srv:result_file-val is deprecated.  Use alicia_flexible_grasp_supervisor-srv:result_file instead.")
  (result_file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TcpCalibrationCommand-response>) ostream)
  "Serializes a message object of type '<TcpCalibrationCommand-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sample_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sample_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sample_count)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sample_count)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'tcp_translation) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'fixed_point) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rms_error_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'max_error_m))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'orientation_span_deg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result_file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TcpCalibrationCommand-response>) istream)
  "Deserializes a message object of type '<TcpCalibrationCommand-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sample_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'sample_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'sample_count)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'sample_count)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'tcp_translation) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'fixed_point) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rms_error_m) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_error_m) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientation_span_deg) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TcpCalibrationCommand-response>)))
  "Returns string type for a service object of type '<TcpCalibrationCommand-response>"
  "alicia_flexible_grasp_supervisor/TcpCalibrationCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TcpCalibrationCommand-response)))
  "Returns string type for a service object of type 'TcpCalibrationCommand-response"
  "alicia_flexible_grasp_supervisor/TcpCalibrationCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TcpCalibrationCommand-response>)))
  "Returns md5sum for a message object of type '<TcpCalibrationCommand-response>"
  "58e49899f0eda63c395f3d3908079771")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TcpCalibrationCommand-response)))
  "Returns md5sum for a message object of type 'TcpCalibrationCommand-response"
  "58e49899f0eda63c395f3d3908079771")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TcpCalibrationCommand-response>)))
  "Returns full string definition for message of type '<TcpCalibrationCommand-response>"
  (cl:format cl:nil "bool success~%string message~%uint32 sample_count~%geometry_msgs/Vector3 tcp_translation~%geometry_msgs/Vector3 fixed_point~%float64 rms_error_m~%float64 max_error_m~%float64 orientation_span_deg~%string result_file~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TcpCalibrationCommand-response)))
  "Returns full string definition for message of type 'TcpCalibrationCommand-response"
  (cl:format cl:nil "bool success~%string message~%uint32 sample_count~%geometry_msgs/Vector3 tcp_translation~%geometry_msgs/Vector3 fixed_point~%float64 rms_error_m~%float64 max_error_m~%float64 orientation_span_deg~%string result_file~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TcpCalibrationCommand-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'tcp_translation))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'fixed_point))
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'result_file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TcpCalibrationCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TcpCalibrationCommand-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':sample_count (sample_count msg))
    (cl:cons ':tcp_translation (tcp_translation msg))
    (cl:cons ':fixed_point (fixed_point msg))
    (cl:cons ':rms_error_m (rms_error_m msg))
    (cl:cons ':max_error_m (max_error_m msg))
    (cl:cons ':orientation_span_deg (orientation_span_deg msg))
    (cl:cons ':result_file (result_file msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TcpCalibrationCommand)))
  'TcpCalibrationCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TcpCalibrationCommand)))
  'TcpCalibrationCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TcpCalibrationCommand)))
  "Returns string type for a service object of type '<TcpCalibrationCommand>"
  "alicia_flexible_grasp_supervisor/TcpCalibrationCommand")