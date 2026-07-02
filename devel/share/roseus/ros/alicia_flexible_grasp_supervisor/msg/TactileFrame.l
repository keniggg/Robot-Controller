;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::TactileFrame)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'TactileFrame (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TACTILEFRAME")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TACTILEFRAME"))

(in-package "ROS")
;;//! \htmlinclude TactileFrame.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass alicia_flexible_grasp_supervisor::TactileFrame
  :super ros::object
  :slots (_header _skin_name _values _rows _cols _total_force_mn _max_force_mn _max_index _center_x _center_y _contact _valid ))

(defmethod alicia_flexible_grasp_supervisor::TactileFrame
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:skin_name __skin_name) "")
    ((:values __values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:rows __rows) 0)
    ((:cols __cols) 0)
    ((:total_force_mn __total_force_mn) 0.0)
    ((:max_force_mn __max_force_mn) 0.0)
    ((:max_index __max_index) 0)
    ((:center_x __center_x) 0.0)
    ((:center_y __center_y) 0.0)
    ((:contact __contact) nil)
    ((:valid __valid) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _skin_name (string __skin_name))
   (setq _values __values)
   (setq _rows (round __rows))
   (setq _cols (round __cols))
   (setq _total_force_mn (float __total_force_mn))
   (setq _max_force_mn (float __max_force_mn))
   (setq _max_index (round __max_index))
   (setq _center_x (float __center_x))
   (setq _center_y (float __center_y))
   (setq _contact __contact)
   (setq _valid __valid)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:skin_name
   (&optional __skin_name)
   (if __skin_name (setq _skin_name __skin_name)) _skin_name)
  (:values
   (&optional __values)
   (if __values (setq _values __values)) _values)
  (:rows
   (&optional __rows)
   (if __rows (setq _rows __rows)) _rows)
  (:cols
   (&optional __cols)
   (if __cols (setq _cols __cols)) _cols)
  (:total_force_mn
   (&optional __total_force_mn)
   (if __total_force_mn (setq _total_force_mn __total_force_mn)) _total_force_mn)
  (:max_force_mn
   (&optional __max_force_mn)
   (if __max_force_mn (setq _max_force_mn __max_force_mn)) _max_force_mn)
  (:max_index
   (&optional __max_index)
   (if __max_index (setq _max_index __max_index)) _max_index)
  (:center_x
   (&optional __center_x)
   (if __center_x (setq _center_x __center_x)) _center_x)
  (:center_y
   (&optional __center_y)
   (if __center_y (setq _center_y __center_y)) _center_y)
  (:contact
   (&optional (__contact :null))
   (if (not (eq __contact :null)) (setq _contact __contact)) _contact)
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _skin_name
    4 (length _skin_name)
    ;; float32[] _values
    (* 4    (length _values)) 4
    ;; uint32 _rows
    4
    ;; uint32 _cols
    4
    ;; float32 _total_force_mn
    4
    ;; float32 _max_force_mn
    4
    ;; uint32 _max_index
    4
    ;; float32 _center_x
    4
    ;; float32 _center_y
    4
    ;; bool _contact
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
     ;; string _skin_name
       (write-long (length _skin_name) s) (princ _skin_name s)
     ;; float32[] _values
     (write-long (length _values) s)
     (dotimes (i (length _values))
       (sys::poke (elt _values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; uint32 _rows
       (write-long _rows s)
     ;; uint32 _cols
       (write-long _cols s)
     ;; float32 _total_force_mn
       (sys::poke _total_force_mn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_force_mn
       (sys::poke _max_force_mn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _max_index
       (write-long _max_index s)
     ;; float32 _center_x
       (sys::poke _center_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _center_y
       (sys::poke _center_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _contact
       (if _contact (write-byte -1 s) (write-byte 0 s))
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _skin_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _skin_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32[] _values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; uint32 _rows
     (setq _rows (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _cols
     (setq _cols (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _total_force_mn
     (setq _total_force_mn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_force_mn
     (setq _max_force_mn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _max_index
     (setq _max_index (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _center_x
     (setq _center_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _center_y
     (setq _center_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _contact
     (setq _contact (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::TactileFrame :md5sum-) "a144ff6b8561cbba1f903254a0728a98")
(setf (get alicia_flexible_grasp_supervisor::TactileFrame :datatype-) "alicia_flexible_grasp_supervisor/TactileFrame")
(setf (get alicia_flexible_grasp_supervisor::TactileFrame :definition-)
      "std_msgs/Header header
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



(provide :alicia_flexible_grasp_supervisor/TactileFrame "a144ff6b8561cbba1f903254a0728a98")


