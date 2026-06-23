;; Auto-generated. Do not edit!


(when (boundp 'alicia_duo_driver::ArmJointState)
  (if (not (find-package "ALICIA_DUO_DRIVER"))
    (make-package "ALICIA_DUO_DRIVER"))
  (shadow 'ArmJointState (find-package "ALICIA_DUO_DRIVER")))
(unless (find-package "ALICIA_DUO_DRIVER::ARMJOINTSTATE")
  (make-package "ALICIA_DUO_DRIVER::ARMJOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude ArmJointState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass alicia_duo_driver::ArmJointState
  :super ros::object
  :slots (_header _joint1 _joint2 _joint3 _joint4 _joint5 _joint6 _gripper _time ))

(defmethod alicia_duo_driver::ArmJointState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint1 __joint1) 0.0)
    ((:joint2 __joint2) 0.0)
    ((:joint3 __joint3) 0.0)
    ((:joint4 __joint4) 0.0)
    ((:joint5 __joint5) 0.0)
    ((:joint6 __joint6) 0.0)
    ((:gripper __gripper) 0.0)
    ((:time __time) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint1 (float __joint1))
   (setq _joint2 (float __joint2))
   (setq _joint3 (float __joint3))
   (setq _joint4 (float __joint4))
   (setq _joint5 (float __joint5))
   (setq _joint6 (float __joint6))
   (setq _gripper (float __gripper))
   (setq _time (float __time))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint1
   (&optional __joint1)
   (if __joint1 (setq _joint1 __joint1)) _joint1)
  (:joint2
   (&optional __joint2)
   (if __joint2 (setq _joint2 __joint2)) _joint2)
  (:joint3
   (&optional __joint3)
   (if __joint3 (setq _joint3 __joint3)) _joint3)
  (:joint4
   (&optional __joint4)
   (if __joint4 (setq _joint4 __joint4)) _joint4)
  (:joint5
   (&optional __joint5)
   (if __joint5 (setq _joint5 __joint5)) _joint5)
  (:joint6
   (&optional __joint6)
   (if __joint6 (setq _joint6 __joint6)) _joint6)
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float32 _joint1
    4
    ;; float32 _joint2
    4
    ;; float32 _joint3
    4
    ;; float32 _joint4
    4
    ;; float32 _joint5
    4
    ;; float32 _joint6
    4
    ;; float32 _gripper
    4
    ;; float32 _time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float32 _joint1
       (sys::poke _joint1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint2
       (sys::poke _joint2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint3
       (sys::poke _joint3 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint4
       (sys::poke _joint4 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint5
       (sys::poke _joint5 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _joint6
       (sys::poke _joint6 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gripper
       (sys::poke _gripper (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _time
       (sys::poke _time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float32 _joint1
     (setq _joint1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint2
     (setq _joint2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint3
     (setq _joint3 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint4
     (setq _joint4 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint5
     (setq _joint5 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _joint6
     (setq _joint6 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gripper
     (setq _gripper (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _time
     (setq _time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get alicia_duo_driver::ArmJointState :md5sum-) "9825570808a8e3729693705ffcdd81b9")
(setf (get alicia_duo_driver::ArmJointState :datatype-) "alicia_duo_driver/ArmJointState")
(setf (get alicia_duo_driver::ArmJointState :definition-)
      "# 标准机械臂关节状态消息 (弧度单位)
# 所有关节角度使用弧度作为单位

Header header

# 六个主要关节角度 (弧度)
float32 joint1
float32 joint2
float32 joint3
float32 joint4
float32 joint5
float32 joint6

# 夹爪角度 (弧度)
float32 gripper

# 可选的运动控制参数
float32 time  # 运动时间(秒)，默认为0表示立即执行
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



(provide :alicia_duo_driver/ArmJointState "9825570808a8e3729693705ffcdd81b9")


