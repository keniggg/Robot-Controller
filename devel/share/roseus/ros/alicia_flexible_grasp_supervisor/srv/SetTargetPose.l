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
  :slots (_pose _execute ))

(defmethod alicia_flexible_grasp_supervisor::SetTargetPoseRequest
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:execute __execute) nil)
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _execute __execute)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:execute
   (&optional (__execute :null))
   (if (not (eq __execute :null)) (setq _execute __execute)) _execute)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetTargetPoseResponse
  :super ros::object
  :slots (_ok _message ))

(defmethod alicia_flexible_grasp_supervisor::SetTargetPoseResponse
  (:init
   (&key
    ((:ok __ok) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _ok __ok)
   (setq _message (string __message))
   self)
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetTargetPose
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::SetTargetPose :md5sum-) "00e64715c2cedd5add3400704911c3af")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPose :datatype-) "alicia_flexible_grasp_supervisor/SetTargetPose")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPose :request) alicia_flexible_grasp_supervisor::SetTargetPoseRequest)
(setf (get alicia_flexible_grasp_supervisor::SetTargetPose :response) alicia_flexible_grasp_supervisor::SetTargetPoseResponse)

(defmethod alicia_flexible_grasp_supervisor::SetTargetPoseRequest
  (:response () (instance alicia_flexible_grasp_supervisor::SetTargetPoseResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseRequest :md5sum-) "00e64715c2cedd5add3400704911c3af")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseRequest :datatype-) "alicia_flexible_grasp_supervisor/SetTargetPoseRequest")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseRequest :definition-)
      "geometry_msgs/Pose pose
bool execute

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
bool ok
string message

")

(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseResponse :md5sum-) "00e64715c2cedd5add3400704911c3af")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseResponse :datatype-) "alicia_flexible_grasp_supervisor/SetTargetPoseResponse")
(setf (get alicia_flexible_grasp_supervisor::SetTargetPoseResponse :definition-)
      "geometry_msgs/Pose pose
bool execute

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
bool ok
string message

")



(provide :alicia_flexible_grasp_supervisor/SetTargetPose "00e64715c2cedd5add3400704911c3af")


