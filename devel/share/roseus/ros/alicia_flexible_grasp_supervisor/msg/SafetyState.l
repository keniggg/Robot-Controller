;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::SafetyState)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'SafetyState (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SAFETYSTATE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SAFETYSTATE"))

(in-package "ROS")
;;//! \htmlinclude SafetyState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass alicia_flexible_grasp_supervisor::SafetyState
  :super ros::object
  :slots (_header _ok _emergency_stop _force_over_limit _robot_timeout _tactile_timeout _camera_timeout _planning_failed _level _message ))

(defmethod alicia_flexible_grasp_supervisor::SafetyState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ok __ok) nil)
    ((:emergency_stop __emergency_stop) nil)
    ((:force_over_limit __force_over_limit) nil)
    ((:robot_timeout __robot_timeout) nil)
    ((:tactile_timeout __tactile_timeout) nil)
    ((:camera_timeout __camera_timeout) nil)
    ((:planning_failed __planning_failed) nil)
    ((:level __level) "")
    ((:message __message) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _ok __ok)
   (setq _emergency_stop __emergency_stop)
   (setq _force_over_limit __force_over_limit)
   (setq _robot_timeout __robot_timeout)
   (setq _tactile_timeout __tactile_timeout)
   (setq _camera_timeout __camera_timeout)
   (setq _planning_failed __planning_failed)
   (setq _level (string __level))
   (setq _message (string __message))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:emergency_stop
   (&optional (__emergency_stop :null))
   (if (not (eq __emergency_stop :null)) (setq _emergency_stop __emergency_stop)) _emergency_stop)
  (:force_over_limit
   (&optional (__force_over_limit :null))
   (if (not (eq __force_over_limit :null)) (setq _force_over_limit __force_over_limit)) _force_over_limit)
  (:robot_timeout
   (&optional (__robot_timeout :null))
   (if (not (eq __robot_timeout :null)) (setq _robot_timeout __robot_timeout)) _robot_timeout)
  (:tactile_timeout
   (&optional (__tactile_timeout :null))
   (if (not (eq __tactile_timeout :null)) (setq _tactile_timeout __tactile_timeout)) _tactile_timeout)
  (:camera_timeout
   (&optional (__camera_timeout :null))
   (if (not (eq __camera_timeout :null)) (setq _camera_timeout __camera_timeout)) _camera_timeout)
  (:planning_failed
   (&optional (__planning_failed :null))
   (if (not (eq __planning_failed :null)) (setq _planning_failed __planning_failed)) _planning_failed)
  (:level
   (&optional __level)
   (if __level (setq _level __level)) _level)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _ok
    1
    ;; bool _emergency_stop
    1
    ;; bool _force_over_limit
    1
    ;; bool _robot_timeout
    1
    ;; bool _tactile_timeout
    1
    ;; bool _camera_timeout
    1
    ;; bool _planning_failed
    1
    ;; string _level
    4 (length _level)
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;; bool _emergency_stop
       (if _emergency_stop (write-byte -1 s) (write-byte 0 s))
     ;; bool _force_over_limit
       (if _force_over_limit (write-byte -1 s) (write-byte 0 s))
     ;; bool _robot_timeout
       (if _robot_timeout (write-byte -1 s) (write-byte 0 s))
     ;; bool _tactile_timeout
       (if _tactile_timeout (write-byte -1 s) (write-byte 0 s))
     ;; bool _camera_timeout
       (if _camera_timeout (write-byte -1 s) (write-byte 0 s))
     ;; bool _planning_failed
       (if _planning_failed (write-byte -1 s) (write-byte 0 s))
     ;; string _level
       (write-long (length _level) s) (princ _level s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _emergency_stop
     (setq _emergency_stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _force_over_limit
     (setq _force_over_limit (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _robot_timeout
     (setq _robot_timeout (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tactile_timeout
     (setq _tactile_timeout (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _camera_timeout
     (setq _camera_timeout (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _planning_failed
     (setq _planning_failed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _level
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _level (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::SafetyState :md5sum-) "d422c2c3e9933ef7c15274804ca0c2f7")
(setf (get alicia_flexible_grasp_supervisor::SafetyState :datatype-) "alicia_flexible_grasp_supervisor/SafetyState")
(setf (get alicia_flexible_grasp_supervisor::SafetyState :definition-)
      "std_msgs/Header header
bool ok
bool emergency_stop
bool force_over_limit
bool robot_timeout
bool tactile_timeout
bool camera_timeout
bool planning_failed
string level
string message

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

")



(provide :alicia_flexible_grasp_supervisor/SafetyState "d422c2c3e9933ef7c15274804ca0c2f7")


