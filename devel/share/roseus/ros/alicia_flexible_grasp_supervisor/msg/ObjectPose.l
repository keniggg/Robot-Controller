;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::ObjectPose)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'ObjectPose (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::OBJECTPOSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::OBJECTPOSE"))

(in-package "ROS")
;;//! \htmlinclude ObjectPose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass alicia_flexible_grasp_supervisor::ObjectPose
  :super ros::object
  :slots (_header _detected _label _confidence _u _v _depth _position_camera _pose_base _status ))

(defmethod alicia_flexible_grasp_supervisor::ObjectPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detected __detected) nil)
    ((:label __label) "")
    ((:confidence __confidence) 0.0)
    ((:u __u) 0)
    ((:v __v) 0)
    ((:depth __depth) 0.0)
    ((:position_camera __position_camera) (instance geometry_msgs::Point :init))
    ((:pose_base __pose_base) (instance geometry_msgs::Pose :init))
    ((:status __status) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _detected __detected)
   (setq _label (string __label))
   (setq _confidence (float __confidence))
   (setq _u (round __u))
   (setq _v (round __v))
   (setq _depth (float __depth))
   (setq _position_camera __position_camera)
   (setq _pose_base __pose_base)
   (setq _status (string __status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detected
   (&optional (__detected :null))
   (if (not (eq __detected :null)) (setq _detected __detected)) _detected)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:u
   (&optional __u)
   (if __u (setq _u __u)) _u)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:position_camera
   (&rest __position_camera)
   (if (keywordp (car __position_camera))
       (send* _position_camera __position_camera)
     (progn
       (if __position_camera (setq _position_camera (car __position_camera)))
       _position_camera)))
  (:pose_base
   (&rest __pose_base)
   (if (keywordp (car __pose_base))
       (send* _pose_base __pose_base)
     (progn
       (if __pose_base (setq _pose_base (car __pose_base)))
       _pose_base)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _detected
    1
    ;; string _label
    4 (length _label)
    ;; float32 _confidence
    4
    ;; uint32 _u
    4
    ;; uint32 _v
    4
    ;; float32 _depth
    4
    ;; geometry_msgs/Point _position_camera
    (send _position_camera :serialization-length)
    ;; geometry_msgs/Pose _pose_base
    (send _pose_base :serialization-length)
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _detected
       (if _detected (write-byte -1 s) (write-byte 0 s))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _u
       (write-long _u s)
     ;; uint32 _v
       (write-long _v s)
     ;; float32 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Point _position_camera
       (send _position_camera :serialize s)
     ;; geometry_msgs/Pose _pose_base
       (send _pose_base :serialize s)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _detected
     (setq _detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _u
     (setq _u (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _v
     (setq _v (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _depth
     (setq _depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Point _position_camera
     (send _position_camera :deserialize buf ptr-) (incf ptr- (send _position_camera :serialization-length))
   ;; geometry_msgs/Pose _pose_base
     (send _pose_base :deserialize buf ptr-) (incf ptr- (send _pose_base :serialization-length))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::ObjectPose :md5sum-) "613afc19135cb8f109a247ba2e6628c9")
(setf (get alicia_flexible_grasp_supervisor::ObjectPose :datatype-) "alicia_flexible_grasp_supervisor/ObjectPose")
(setf (get alicia_flexible_grasp_supervisor::ObjectPose :definition-)
      "std_msgs/Header header
bool detected
string label
float32 confidence
uint32 u
uint32 v
float32 depth
geometry_msgs/Point position_camera
geometry_msgs/Pose pose_base
string status

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :alicia_flexible_grasp_supervisor/ObjectPose "613afc19135cb8f109a247ba2e6628c9")


