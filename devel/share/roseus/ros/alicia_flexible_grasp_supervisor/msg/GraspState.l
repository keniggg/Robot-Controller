;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::GraspState)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'GraspState (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))

(in-package "ROS")
;;//! \htmlinclude GraspState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*IDLE*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*IDLE* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*IDLE* 0)
(intern "*SEARCH_OBJECT*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*SEARCH_OBJECT* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*SEARCH_OBJECT* 1)
(intern "*ESTIMATE_POSE*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*ESTIMATE_POSE* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*ESTIMATE_POSE* 2)
(intern "*PLAN_PREGRASP*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*PLAN_PREGRASP* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*PLAN_PREGRASP* 3)
(intern "*MOVE_PREGRASP*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*MOVE_PREGRASP* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*MOVE_PREGRASP* 4)
(intern "*APPROACH_OBJECT*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*APPROACH_OBJECT* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*APPROACH_OBJECT* 5)
(intern "*COMPLIANT_CLOSE*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*COMPLIANT_CLOSE* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*COMPLIANT_CLOSE* 6)
(intern "*GRASP_VERIFY*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*GRASP_VERIFY* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*GRASP_VERIFY* 7)
(intern "*LIFT_OBJECT*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*LIFT_OBJECT* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*LIFT_OBJECT* 8)
(intern "*PLACE_OBJECT*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*PLACE_OBJECT* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*PLACE_OBJECT* 9)
(intern "*RELEASE_OBJECT*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*RELEASE_OBJECT* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*RELEASE_OBJECT* 10)
(intern "*SUCCESS*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*SUCCESS* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*SUCCESS* 11)
(intern "*FAILED*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*FAILED* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*FAILED* 12)
(intern "*EMERGENCY_STOP*" (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(shadow '*EMERGENCY_STOP* (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))
(defconstant alicia_flexible_grasp_supervisor::GraspState::*EMERGENCY_STOP* 13)

(defun alicia_flexible_grasp_supervisor::GraspState-to-symbol (const)
  (cond
        ((= const 0) 'alicia_flexible_grasp_supervisor::GraspState::*IDLE*)
        ((= const 1) 'alicia_flexible_grasp_supervisor::GraspState::*SEARCH_OBJECT*)
        ((= const 2) 'alicia_flexible_grasp_supervisor::GraspState::*ESTIMATE_POSE*)
        ((= const 3) 'alicia_flexible_grasp_supervisor::GraspState::*PLAN_PREGRASP*)
        ((= const 4) 'alicia_flexible_grasp_supervisor::GraspState::*MOVE_PREGRASP*)
        ((= const 5) 'alicia_flexible_grasp_supervisor::GraspState::*APPROACH_OBJECT*)
        ((= const 6) 'alicia_flexible_grasp_supervisor::GraspState::*COMPLIANT_CLOSE*)
        ((= const 7) 'alicia_flexible_grasp_supervisor::GraspState::*GRASP_VERIFY*)
        ((= const 8) 'alicia_flexible_grasp_supervisor::GraspState::*LIFT_OBJECT*)
        ((= const 9) 'alicia_flexible_grasp_supervisor::GraspState::*PLACE_OBJECT*)
        ((= const 10) 'alicia_flexible_grasp_supervisor::GraspState::*RELEASE_OBJECT*)
        ((= const 11) 'alicia_flexible_grasp_supervisor::GraspState::*SUCCESS*)
        ((= const 12) 'alicia_flexible_grasp_supervisor::GraspState::*FAILED*)
        ((= const 13) 'alicia_flexible_grasp_supervisor::GraspState::*EMERGENCY_STOP*)
        (t nil)))

(defclass alicia_flexible_grasp_supervisor::GraspState
  :super ros::object
  :slots (_header _state _state_name _running _success _message _current_force _target_force _object_pose_base _target_pose ))

(defmethod alicia_flexible_grasp_supervisor::GraspState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:state_name __state_name) "")
    ((:running __running) nil)
    ((:success __success) nil)
    ((:message __message) "")
    ((:current_force __current_force) 0.0)
    ((:target_force __target_force) 0.0)
    ((:object_pose_base __object_pose_base) (instance geometry_msgs::Pose :init))
    ((:target_pose __target_pose) (instance geometry_msgs::Pose :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _state_name (string __state_name))
   (setq _running __running)
   (setq _success __success)
   (setq _message (string __message))
   (setq _current_force (float __current_force))
   (setq _target_force (float __target_force))
   (setq _object_pose_base __object_pose_base)
   (setq _target_pose __target_pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:state_name
   (&optional __state_name)
   (if __state_name (setq _state_name __state_name)) _state_name)
  (:running
   (&optional (__running :null))
   (if (not (eq __running :null)) (setq _running __running)) _running)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:current_force
   (&optional __current_force)
   (if __current_force (setq _current_force __current_force)) _current_force)
  (:target_force
   (&optional __target_force)
   (if __target_force (setq _target_force __target_force)) _target_force)
  (:object_pose_base
   (&rest __object_pose_base)
   (if (keywordp (car __object_pose_base))
       (send* _object_pose_base __object_pose_base)
     (progn
       (if __object_pose_base (setq _object_pose_base (car __object_pose_base)))
       _object_pose_base)))
  (:target_pose
   (&rest __target_pose)
   (if (keywordp (car __target_pose))
       (send* _target_pose __target_pose)
     (progn
       (if __target_pose (setq _target_pose (car __target_pose)))
       _target_pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _state
    1
    ;; string _state_name
    4 (length _state_name)
    ;; bool _running
    1
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; float32 _current_force
    4
    ;; float32 _target_force
    4
    ;; geometry_msgs/Pose _object_pose_base
    (send _object_pose_base :serialization-length)
    ;; geometry_msgs/Pose _target_pose
    (send _target_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _state
       (write-byte _state s)
     ;; string _state_name
       (write-long (length _state_name) s) (princ _state_name s)
     ;; bool _running
       (if _running (write-byte -1 s) (write-byte 0 s))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; float32 _current_force
       (sys::poke _current_force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_force
       (sys::poke _target_force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Pose _object_pose_base
       (send _object_pose_base :serialize s)
     ;; geometry_msgs/Pose _target_pose
       (send _target_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _state_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _running
     (setq _running (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _current_force
     (setq _current_force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_force
     (setq _target_force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Pose _object_pose_base
     (send _object_pose_base :deserialize buf ptr-) (incf ptr- (send _object_pose_base :serialization-length))
   ;; geometry_msgs/Pose _target_pose
     (send _target_pose :deserialize buf ptr-) (incf ptr- (send _target_pose :serialization-length))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::GraspState :md5sum-) "75215c45032076e51dbd82164d6951cb")
(setf (get alicia_flexible_grasp_supervisor::GraspState :datatype-) "alicia_flexible_grasp_supervisor/GraspState")
(setf (get alicia_flexible_grasp_supervisor::GraspState :definition-)
      "std_msgs/Header header
uint8 IDLE=0
uint8 SEARCH_OBJECT=1
uint8 ESTIMATE_POSE=2
uint8 PLAN_PREGRASP=3
uint8 MOVE_PREGRASP=4
uint8 APPROACH_OBJECT=5
uint8 COMPLIANT_CLOSE=6
uint8 GRASP_VERIFY=7
uint8 LIFT_OBJECT=8
uint8 PLACE_OBJECT=9
uint8 RELEASE_OBJECT=10
uint8 SUCCESS=11
uint8 FAILED=12
uint8 EMERGENCY_STOP=13
uint8 state
string state_name
bool running
bool success
string message
float32 current_force
float32 target_force
geometry_msgs/Pose object_pose_base
geometry_msgs/Pose target_pose

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

")



(provide :alicia_flexible_grasp_supervisor/GraspState "75215c45032076e51dbd82164d6951cb")


