; Auto-generated. Do not edit!


(cl:in-package alicia_flexible_grasp_supervisor-msg)


;//! \htmlinclude ObjectPose.msg.html

(cl:defclass <ObjectPose> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detected
    :reader detected
    :initarg :detected
    :type cl:boolean
    :initform cl:nil)
   (label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (u
    :reader u
    :initarg :u
    :type cl:integer
    :initform 0)
   (v
    :reader v
    :initarg :v
    :type cl:integer
    :initform 0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0)
   (position_camera
    :reader position_camera
    :initarg :position_camera
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (pose_base
    :reader pose_base
    :initarg :pose_base
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass ObjectPose (<ObjectPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name alicia_flexible_grasp_supervisor-msg:<ObjectPose> is deprecated: use alicia_flexible_grasp_supervisor-msg:ObjectPose instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:header-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detected-val :lambda-list '(m))
(cl:defmethod detected-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:detected-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:detected instead.")
  (detected m))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:label-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:confidence-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'u-val :lambda-list '(m))
(cl:defmethod u-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:u-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:u instead.")
  (u m))

(cl:ensure-generic-function 'v-val :lambda-list '(m))
(cl:defmethod v-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:v-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:v instead.")
  (v m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:depth-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:depth instead.")
  (depth m))

(cl:ensure-generic-function 'position_camera-val :lambda-list '(m))
(cl:defmethod position_camera-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:position_camera-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:position_camera instead.")
  (position_camera m))

(cl:ensure-generic-function 'pose_base-val :lambda-list '(m))
(cl:defmethod pose_base-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:pose_base-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:pose_base instead.")
  (pose_base m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ObjectPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader alicia_flexible_grasp_supervisor-msg:status-val is deprecated.  Use alicia_flexible_grasp_supervisor-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectPose>) ostream)
  "Serializes a message object of type '<ObjectPose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'detected) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'u)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'u)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'v)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'v)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'v)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'v)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position_camera) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose_base) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectPose>) istream)
  "Deserializes a message object of type '<ObjectPose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'u)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'u)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'u)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'u)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'v)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'v)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'v)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'v)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position_camera) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose_base) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectPose>)))
  "Returns string type for a message object of type '<ObjectPose>"
  "alicia_flexible_grasp_supervisor/ObjectPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectPose)))
  "Returns string type for a message object of type 'ObjectPose"
  "alicia_flexible_grasp_supervisor/ObjectPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectPose>)))
  "Returns md5sum for a message object of type '<ObjectPose>"
  "613afc19135cb8f109a247ba2e6628c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectPose)))
  "Returns md5sum for a message object of type 'ObjectPose"
  "613afc19135cb8f109a247ba2e6628c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectPose>)))
  "Returns full string definition for message of type '<ObjectPose>"
  (cl:format cl:nil "std_msgs/Header header~%bool detected~%string label~%float32 confidence~%uint32 u~%uint32 v~%float32 depth~%geometry_msgs/Point position_camera~%geometry_msgs/Pose pose_base~%string status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectPose)))
  "Returns full string definition for message of type 'ObjectPose"
  (cl:format cl:nil "std_msgs/Header header~%bool detected~%string label~%float32 confidence~%uint32 u~%uint32 v~%float32 depth~%geometry_msgs/Point position_camera~%geometry_msgs/Pose pose_base~%string status~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectPose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'label))
     4
     4
     4
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position_camera))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose_base))
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectPose>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectPose
    (cl:cons ':header (header msg))
    (cl:cons ':detected (detected msg))
    (cl:cons ':label (label msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':u (u msg))
    (cl:cons ':v (v msg))
    (cl:cons ':depth (depth msg))
    (cl:cons ':position_camera (position_camera msg))
    (cl:cons ':pose_base (pose_base msg))
    (cl:cons ':status (status msg))
))
