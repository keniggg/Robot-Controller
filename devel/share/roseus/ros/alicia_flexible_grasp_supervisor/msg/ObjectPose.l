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
  :slots (_header _detected _label _confidence _u _v _bbox_x _bbox_y _bbox_width _bbox_height _depth_m _pose_camera _pose_base ))

(defmethod alicia_flexible_grasp_supervisor::ObjectPose
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detected __detected) nil)
    ((:label __label) "")
    ((:confidence __confidence) 0.0)
    ((:u __u) 0)
    ((:v __v) 0)
    ((:bbox_x __bbox_x) 0)
    ((:bbox_y __bbox_y) 0)
    ((:bbox_width __bbox_width) 0)
    ((:bbox_height __bbox_height) 0)
    ((:depth_m __depth_m) 0.0)
    ((:pose_camera __pose_camera) (instance geometry_msgs::PoseStamped :init))
    ((:pose_base __pose_base) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _detected __detected)
   (setq _label (string __label))
   (setq _confidence (float __confidence))
   (setq _u (round __u))
   (setq _v (round __v))
   (setq _bbox_x (round __bbox_x))
   (setq _bbox_y (round __bbox_y))
   (setq _bbox_width (round __bbox_width))
   (setq _bbox_height (round __bbox_height))
   (setq _depth_m (float __depth_m))
   (setq _pose_camera __pose_camera)
   (setq _pose_base __pose_base)
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
  (:bbox_x
   (&optional __bbox_x)
   (if __bbox_x (setq _bbox_x __bbox_x)) _bbox_x)
  (:bbox_y
   (&optional __bbox_y)
   (if __bbox_y (setq _bbox_y __bbox_y)) _bbox_y)
  (:bbox_width
   (&optional __bbox_width)
   (if __bbox_width (setq _bbox_width __bbox_width)) _bbox_width)
  (:bbox_height
   (&optional __bbox_height)
   (if __bbox_height (setq _bbox_height __bbox_height)) _bbox_height)
  (:depth_m
   (&optional __depth_m)
   (if __depth_m (setq _depth_m __depth_m)) _depth_m)
  (:pose_camera
   (&rest __pose_camera)
   (if (keywordp (car __pose_camera))
       (send* _pose_camera __pose_camera)
     (progn
       (if __pose_camera (setq _pose_camera (car __pose_camera)))
       _pose_camera)))
  (:pose_base
   (&rest __pose_base)
   (if (keywordp (car __pose_base))
       (send* _pose_base __pose_base)
     (progn
       (if __pose_base (setq _pose_base (car __pose_base)))
       _pose_base)))
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
    ;; uint32 _bbox_x
    4
    ;; uint32 _bbox_y
    4
    ;; uint32 _bbox_width
    4
    ;; uint32 _bbox_height
    4
    ;; float32 _depth_m
    4
    ;; geometry_msgs/PoseStamped _pose_camera
    (send _pose_camera :serialization-length)
    ;; geometry_msgs/PoseStamped _pose_base
    (send _pose_base :serialization-length)
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
     ;; uint32 _bbox_x
       (write-long _bbox_x s)
     ;; uint32 _bbox_y
       (write-long _bbox_y s)
     ;; uint32 _bbox_width
       (write-long _bbox_width s)
     ;; uint32 _bbox_height
       (write-long _bbox_height s)
     ;; float32 _depth_m
       (sys::poke _depth_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/PoseStamped _pose_camera
       (send _pose_camera :serialize s)
     ;; geometry_msgs/PoseStamped _pose_base
       (send _pose_base :serialize s)
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
   ;; uint32 _bbox_x
     (setq _bbox_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _bbox_y
     (setq _bbox_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _bbox_width
     (setq _bbox_width (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _bbox_height
     (setq _bbox_height (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _depth_m
     (setq _depth_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/PoseStamped _pose_camera
     (send _pose_camera :deserialize buf ptr-) (incf ptr- (send _pose_camera :serialization-length))
   ;; geometry_msgs/PoseStamped _pose_base
     (send _pose_base :deserialize buf ptr-) (incf ptr- (send _pose_base :serialization-length))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::ObjectPose :md5sum-) "2d3731316703d38acb9f83ae0dd46e90")
(setf (get alicia_flexible_grasp_supervisor::ObjectPose :datatype-) "alicia_flexible_grasp_supervisor/ObjectPose")
(setf (get alicia_flexible_grasp_supervisor::ObjectPose :definition-)
      "std_msgs/Header header
bool detected
string label
float32 confidence
uint32 u
uint32 v
uint32 bbox_x
uint32 bbox_y
uint32 bbox_width
uint32 bbox_height
float32 depth_m
geometry_msgs/PoseStamped pose_camera
geometry_msgs/PoseStamped pose_base

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
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :alicia_flexible_grasp_supervisor/ObjectPose "2d3731316703d38acb9f83ae0dd46e90")


