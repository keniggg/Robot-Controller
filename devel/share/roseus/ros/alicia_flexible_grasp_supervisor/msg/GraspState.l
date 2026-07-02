;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::GraspState)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'GraspState (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASPSTATE"))

(in-package "ROS")
;;//! \htmlinclude GraspState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass alicia_flexible_grasp_supervisor::GraspState
  :super ros::object
  :slots (_header _state _stage _active _success _message ))

(defmethod alicia_flexible_grasp_supervisor::GraspState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) "")
    ((:stage __stage) 0)
    ((:active __active) nil)
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (string __state))
   (setq _stage (round __stage))
   (setq _active __active)
   (setq _success __success)
   (setq _message (string __message))
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
  (:stage
   (&optional __stage)
   (if __stage (setq _stage __stage)) _stage)
  (:active
   (&optional (__active :null))
   (if (not (eq __active :null)) (setq _active __active)) _active)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _state
    4 (length _state)
    ;; int32 _stage
    4
    ;; bool _active
    1
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; int32 _stage
       (write-long _stage s)
     ;; bool _active
       (if _active (write-byte -1 s) (write-byte 0 s))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _stage
     (setq _stage (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _active
     (setq _active (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::GraspState :md5sum-) "c85c0a7de717463c53d5cec2b43693e8")
(setf (get alicia_flexible_grasp_supervisor::GraspState :datatype-) "alicia_flexible_grasp_supervisor/GraspState")
(setf (get alicia_flexible_grasp_supervisor::GraspState :definition-)
      "std_msgs/Header header
string state
int32 stage
bool active
bool success
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



(provide :alicia_flexible_grasp_supervisor/GraspState "c85c0a7de717463c53d5cec2b43693e8")


