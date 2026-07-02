;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::SetTargetPose)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'SetTargetPose (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETTARGETPOSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETTARGETPOSE"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETTARGETPOSEREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETTARGETPOSEREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETTARGETPOSERESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETTARGETPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass alicia_flexible_grasp_supervisor::SetTargetPoseRequest
  :super ros::object
  :slots (_target _execute ))

(defmethod alicia_flexible_grasp_supervisor::SetTargetPoseRequest
  (:init
   (&key
    ((:target __target) (instance geometry_msgs::PoseStamped :init))
    ((:execute __execute) nil)
    )
   (send-super :init)
   (setq _target __target)
   (setq _execute __execute)
   self)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:execute
   (&optional (__execute :null))
   (if (not (eq __execute :null)) (setq _execute __execute)) _execute)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _target
    (send _target :serialization-length)
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _target
       (send _target :serialize s)
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetTargetPoseResponse
  :super ros::object
  :slots (_success _message ))

(defmethod alicia_flexible_grasp_supervisor::SetTargetPoseResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetTargetPose
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::SetTargetPose :md5sum-) "2cf3c52fcc5b5b6cce00c68f65c0b916")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPose :datatype-) "alicia_flexible_grasp_supervisor/SetTargetPose")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPose :request) alicia_flexible_grasp_supervisor::SetTargetPoseRequest)
(setf (get alicia_flexible_grasp_supervisor::SetTargetPose :response) alicia_flexible_grasp_supervisor::SetTargetPoseResponse)

(defmethod alicia_flexible_grasp_supervisor::SetTargetPoseRequest
  (:response () (instance alicia_flexible_grasp_supervisor::SetTargetPoseResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseRequest :md5sum-) "2cf3c52fcc5b5b6cce00c68f65c0b916")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseRequest :datatype-) "alicia_flexible_grasp_supervisor/SetTargetPoseRequest")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseRequest :definition-)
      "geometry_msgs/PoseStamped target
bool execute

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
---
bool success
string message

")

(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseResponse :md5sum-) "2cf3c52fcc5b5b6cce00c68f65c0b916")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseResponse :datatype-) "alicia_flexible_grasp_supervisor/SetTargetPoseResponse")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseResponse :definition-)
      "geometry_msgs/PoseStamped target
bool execute

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
---
bool success
string message

")



(provide :alicia_flexible_grasp_supervisor/SetTargetPose "2cf3c52fcc5b5b6cce00c68f65c0b916")


