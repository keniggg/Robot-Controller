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
  :slots (_header _skin_id _rows _cols _values _total_force _max_force _max_index _center_x _center_y _contact _valid _status ))

(defmethod alicia_flexible_grasp_supervisor::TactileFrame
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:skin_id __skin_id) 0)
    ((:rows __rows) 0)
    ((:cols __cols) 0)
    ((:values __values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:total_force __total_force) 0.0)
    ((:max_force __max_force) 0.0)
    ((:max_index __max_index) 0)
    ((:center_x __center_x) 0.0)
    ((:center_y __center_y) 0.0)
    ((:contact __contact) nil)
    ((:valid __valid) nil)
    ((:status __status) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _skin_id (round __skin_id))
   (setq _rows (round __rows))
   (setq _cols (round __cols))
   (setq _values __values)
   (setq _total_force (float __total_force))
   (setq _max_force (float __max_force))
   (setq _max_index (round __max_index))
   (setq _center_x (float __center_x))
   (setq _center_y (float __center_y))
   (setq _contact __contact)
   (setq _valid __valid)
   (setq _status (string __status))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:skin_id
   (&optional __skin_id)
   (if __skin_id (setq _skin_id __skin_id)) _skin_id)
  (:rows
   (&optional __rows)
   (if __rows (setq _rows __rows)) _rows)
  (:cols
   (&optional __cols)
   (if __cols (setq _cols __cols)) _cols)
  (:values
   (&optional __values)
   (if __values (setq _values __values)) _values)
  (:total_force
   (&optional __total_force)
   (if __total_force (setq _total_force __total_force)) _total_force)
  (:max_force
   (&optional __max_force)
   (if __max_force (setq _max_force __max_force)) _max_force)
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
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _skin_id
    1
    ;; uint16 _rows
    2
    ;; uint16 _cols
    2
    ;; float32[] _values
    (* 4    (length _values)) 4
    ;; float32 _total_force
    4
    ;; float32 _max_force
    4
    ;; uint16 _max_index
    2
    ;; float32 _center_x
    4
    ;; float32 _center_y
    4
    ;; bool _contact
    1
    ;; bool _valid
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
     ;; uint8 _skin_id
       (write-byte _skin_id s)
     ;; uint16 _rows
       (write-word _rows s)
     ;; uint16 _cols
       (write-word _cols s)
     ;; float32[] _values
     (write-long (length _values) s)
     (dotimes (i (length _values))
       (sys::poke (elt _values i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _total_force
       (sys::poke _total_force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_force
       (sys::poke _max_force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint16 _max_index
       (write-word _max_index s)
     ;; float32 _center_x
       (sys::poke _center_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _center_y
       (sys::poke _center_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _contact
       (if _contact (write-byte -1 s) (write-byte 0 s))
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _skin_id
     (setq _skin_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _rows
     (setq _rows (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _cols
     (setq _cols (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32[] _values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _values i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _total_force
     (setq _total_force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_force
     (setq _max_force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint16 _max_index
     (setq _max_index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _center_x
     (setq _center_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _center_y
     (setq _center_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _contact
     (setq _contact (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::TactileFrame :md5sum-) "02ca98ed2c6b94321c339531010cfc87")
(setf (get alicia_flexible_grasp_supervisor::TactileFrame :datatype-) "alicia_flexible_grasp_supervisor/TactileFrame")
(setf (get alicia_flexible_grasp_supervisor::TactileFrame :definition-)
      "std_msgs/Header header
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



(provide :alicia_flexible_grasp_supervisor/TactileFrame "02ca98ed2c6b94321c339531010cfc87")


