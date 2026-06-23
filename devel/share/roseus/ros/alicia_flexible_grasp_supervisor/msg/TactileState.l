;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::TactileState)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'TactileState (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TACTILESTATE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TACTILESTATE"))

(in-package "ROS")
;;//! \htmlinclude TactileState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass alicia_flexible_grasp_supervisor::TactileState
  :super ros::object
  :slots (_header _skin1 _skin2 _total_grip_force _force_diff _left_contact _right_contact _object_grasped _slip_detected _status ))

(defmethod alicia_flexible_grasp_supervisor::TactileState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:skin1 __skin1) (instance alicia_flexible_grasp_supervisor::TactileFrame :init))
    ((:skin2 __skin2) (instance alicia_flexible_grasp_supervisor::TactileFrame :init))
    ((:total_grip_force __total_grip_force) 0.0)
    ((:force_diff __force_diff) 0.0)
    ((:left_contact __left_contact) nil)
    ((:right_contact __right_contact) nil)
    ((:object_grasped __object_grasped) nil)
    ((:slip_detected __slip_detected) nil)
    ((:status __status) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _skin1 __skin1)
   (setq _skin2 __skin2)
   (setq _total_grip_force (float __total_grip_force))
   (setq _force_diff (float __force_diff))
   (setq _left_contact __left_contact)
   (setq _right_contact __right_contact)
   (setq _object_grasped __object_grasped)
   (setq _slip_detected __slip_detected)
   (setq _status (string __status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:skin1
   (&rest __skin1)
   (if (keywordp (car __skin1))
       (send* _skin1 __skin1)
     (progn
       (if __skin1 (setq _skin1 (car __skin1)))
       _skin1)))
  (:skin2
   (&rest __skin2)
   (if (keywordp (car __skin2))
       (send* _skin2 __skin2)
     (progn
       (if __skin2 (setq _skin2 (car __skin2)))
       _skin2)))
  (:total_grip_force
   (&optional __total_grip_force)
   (if __total_grip_force (setq _total_grip_force __total_grip_force)) _total_grip_force)
  (:force_diff
   (&optional __force_diff)
   (if __force_diff (setq _force_diff __force_diff)) _force_diff)
  (:left_contact
   (&optional (__left_contact :null))
   (if (not (eq __left_contact :null)) (setq _left_contact __left_contact)) _left_contact)
  (:right_contact
   (&optional (__right_contact :null))
   (if (not (eq __right_contact :null)) (setq _right_contact __right_contact)) _right_contact)
  (:object_grasped
   (&optional (__object_grasped :null))
   (if (not (eq __object_grasped :null)) (setq _object_grasped __object_grasped)) _object_grasped)
  (:slip_detected
   (&optional (__slip_detected :null))
   (if (not (eq __slip_detected :null)) (setq _slip_detected __slip_detected)) _slip_detected)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; alicia_flexible_grasp_supervisor/TactileFrame _skin1
    (send _skin1 :serialization-length)
    ;; alicia_flexible_grasp_supervisor/TactileFrame _skin2
    (send _skin2 :serialization-length)
    ;; float32 _total_grip_force
    4
    ;; float32 _force_diff
    4
    ;; bool _left_contact
    1
    ;; bool _right_contact
    1
    ;; bool _object_grasped
    1
    ;; bool _slip_detected
    1
    ;; string _status
    4 (length _status)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; alicia_flexible_grasp_supervisor/TactileFrame _skin1
       (send _skin1 :serialize s)
     ;; alicia_flexible_grasp_supervisor/TactileFrame _skin2
       (send _skin2 :serialize s)
     ;; float32 _total_grip_force
       (sys::poke _total_grip_force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force_diff
       (sys::poke _force_diff (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _left_contact
       (if _left_contact (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_contact
       (if _right_contact (write-byte -1 s) (write-byte 0 s))
     ;; bool _object_grasped
       (if _object_grasped (write-byte -1 s) (write-byte 0 s))
     ;; bool _slip_detected
       (if _slip_detected (write-byte -1 s) (write-byte 0 s))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; alicia_flexible_grasp_supervisor/TactileFrame _skin1
     (send _skin1 :deserialize buf ptr-) (incf ptr- (send _skin1 :serialization-length))
   ;; alicia_flexible_grasp_supervisor/TactileFrame _skin2
     (send _skin2 :deserialize buf ptr-) (incf ptr- (send _skin2 :serialization-length))
   ;; float32 _total_grip_force
     (setq _total_grip_force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force_diff
     (setq _force_diff (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _left_contact
     (setq _left_contact (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_contact
     (setq _right_contact (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _object_grasped
     (setq _object_grasped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _slip_detected
     (setq _slip_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::TactileState :md5sum-) "9b76704935dbe875f8fc5a941f64277a")
(setf (get alicia_flexible_grasp_supervisor::TactileState :datatype-) "alicia_flexible_grasp_supervisor/TactileState")
(setf (get alicia_flexible_grasp_supervisor::TactileState :definition-)
      "std_msgs/Header header
alicia_flexible_grasp_supervisor/TactileFrame skin1
alicia_flexible_grasp_supervisor/TactileFrame skin2
float32 total_grip_force
float32 force_diff
bool left_contact
bool right_contact
bool object_grasped
bool slip_detected
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
MSG: alicia_flexible_grasp_supervisor/TactileFrame
std_msgs/Header header
uint8 skin_id
uint16 rows
uint16 cols
float32[] values
float32 total_force
float32 max_force
uint16 max_index
float32 center_x
float32 center_y
bool contact
bool valid
string status

")



(provide :alicia_flexible_grasp_supervisor/TactileState "9b76704935dbe875f8fc5a941f64277a")


