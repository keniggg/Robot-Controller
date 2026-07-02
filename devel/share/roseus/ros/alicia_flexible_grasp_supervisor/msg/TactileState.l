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
  :slots (_header _left _right _total_grip_force_mn _force_diff_mn _left_contact _right_contact _object_grasped _slip_detected _valid ))

(defmethod alicia_flexible_grasp_supervisor::TactileState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:left __left) (instance alicia_flexible_grasp_supervisor::TactileFrame :init))
    ((:right __right) (instance alicia_flexible_grasp_supervisor::TactileFrame :init))
    ((:total_grip_force_mn __total_grip_force_mn) 0.0)
    ((:force_diff_mn __force_diff_mn) 0.0)
    ((:left_contact __left_contact) nil)
    ((:right_contact __right_contact) nil)
    ((:object_grasped __object_grasped) nil)
    ((:slip_detected __slip_detected) nil)
    ((:valid __valid) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _left __left)
   (setq _right __right)
   (setq _total_grip_force_mn (float __total_grip_force_mn))
   (setq _force_diff_mn (float __force_diff_mn))
   (setq _left_contact __left_contact)
   (setq _right_contact __right_contact)
   (setq _object_grasped __object_grasped)
   (setq _slip_detected __slip_detected)
   (setq _valid __valid)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:left
   (&rest __left)
   (if (keywordp (car __left))
       (send* _left __left)
     (progn
       (if __left (setq _left (car __left)))
       _left)))
  (:right
   (&rest __right)
   (if (keywordp (car __right))
       (send* _right __right)
     (progn
       (if __right (setq _right (car __right)))
       _right)))
  (:total_grip_force_mn
   (&optional __total_grip_force_mn)
   (if __total_grip_force_mn (setq _total_grip_force_mn __total_grip_force_mn)) _total_grip_force_mn)
  (:force_diff_mn
   (&optional __force_diff_mn)
   (if __force_diff_mn (setq _force_diff_mn __force_diff_mn)) _force_diff_mn)
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
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; alicia_flexible_grasp_supervisor/TactileFrame _left
    (send _left :serialization-length)
    ;; alicia_flexible_grasp_supervisor/TactileFrame _right
    (send _right :serialization-length)
    ;; float32 _total_grip_force_mn
    4
    ;; float32 _force_diff_mn
    4
    ;; bool _left_contact
    1
    ;; bool _right_contact
    1
    ;; bool _object_grasped
    1
    ;; bool _slip_detected
    1
    ;; bool _valid
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; alicia_flexible_grasp_supervisor/TactileFrame _left
       (send _left :serialize s)
     ;; alicia_flexible_grasp_supervisor/TactileFrame _right
       (send _right :serialize s)
     ;; float32 _total_grip_force_mn
       (sys::poke _total_grip_force_mn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _force_diff_mn
       (sys::poke _force_diff_mn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _left_contact
       (if _left_contact (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_contact
       (if _right_contact (write-byte -1 s) (write-byte 0 s))
     ;; bool _object_grasped
       (if _object_grasped (write-byte -1 s) (write-byte 0 s))
     ;; bool _slip_detected
       (if _slip_detected (write-byte -1 s) (write-byte 0 s))
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; alicia_flexible_grasp_supervisor/TactileFrame _left
     (send _left :deserialize buf ptr-) (incf ptr- (send _left :serialization-length))
   ;; alicia_flexible_grasp_supervisor/TactileFrame _right
     (send _right :deserialize buf ptr-) (incf ptr- (send _right :serialization-length))
   ;; float32 _total_grip_force_mn
     (setq _total_grip_force_mn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _force_diff_mn
     (setq _force_diff_mn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _left_contact
     (setq _left_contact (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_contact
     (setq _right_contact (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _object_grasped
     (setq _object_grasped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _slip_detected
     (setq _slip_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::TactileState :md5sum-) "5df6a8c43bd865ec0ec8d2f74fe1aa66")
(setf (get alicia_flexible_grasp_supervisor::TactileState :datatype-) "alicia_flexible_grasp_supervisor/TactileState")
(setf (get alicia_flexible_grasp_supervisor::TactileState :definition-)
      "std_msgs/Header header
alicia_flexible_grasp_supervisor/TactileFrame left
alicia_flexible_grasp_supervisor/TactileFrame right
float32 total_grip_force_mn
float32 force_diff_mn
bool left_contact
bool right_contact
bool object_grasped
bool slip_detected
bool valid

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
string skin_name
float32[] values
uint32 rows
uint32 cols
float32 total_force_mn
float32 max_force_mn
uint32 max_index
float32 center_x
float32 center_y
bool contact
bool valid

")



(provide :alicia_flexible_grasp_supervisor/TactileState "5df6a8c43bd865ec0ec8d2f74fe1aa66")


