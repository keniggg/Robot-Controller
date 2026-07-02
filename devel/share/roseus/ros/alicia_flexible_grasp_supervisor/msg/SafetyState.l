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
  :slots (_header _ok _emergency_stop _level _message ))

(defmethod alicia_flexible_grasp_supervisor::SafetyState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ok __ok) nil)
    ((:emergency_stop __emergency_stop) nil)
    ((:level __level) "")
    ((:message __message) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _ok __ok)
   (setq _emergency_stop __emergency_stop)
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
   ;; string _level
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _level (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::SafetyState :md5sum-) "b5ace411f24918274b9d45871c146d30")
(setf (get alicia_flexible_grasp_supervisor::SafetyState :datatype-) "alicia_flexible_grasp_supervisor/SafetyState")
(setf (get alicia_flexible_grasp_supervisor::SafetyState :definition-)
      "std_msgs/Header header
bool ok
bool emergency_stop
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



(provide :alicia_flexible_grasp_supervisor/SafetyState "b5ace411f24918274b9d45871c146d30")


