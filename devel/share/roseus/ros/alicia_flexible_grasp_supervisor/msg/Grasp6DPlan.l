;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::Grasp6DPlan)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'Grasp6DPlan (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASP6DPLAN")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::GRASP6DPLAN"))

(in-package "ROS")
;;//! \htmlinclude Grasp6DPlan.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass alicia_flexible_grasp_supervisor::Grasp6DPlan
  :super ros::object
  :slots (_header _valid _poses _score _candidate_width_m _required_open_width_m _object_geometry _model_choice _plan_id _diagnostic ))

(defmethod alicia_flexible_grasp_supervisor::Grasp6DPlan
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:valid __valid) nil)
    ((:poses __poses) ())
    ((:score __score) 0.0)
    ((:candidate_width_m __candidate_width_m) 0.0)
    ((:required_open_width_m __required_open_width_m) 0.0)
    ((:object_geometry __object_geometry) (instance alicia_flexible_grasp_supervisor::ObjectGeometry :init))
    ((:model_choice __model_choice) "")
    ((:plan_id __plan_id) "")
    ((:diagnostic __diagnostic) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _valid __valid)
   (setq _poses __poses)
   (setq _score (float __score))
   (setq _candidate_width_m (float __candidate_width_m))
   (setq _required_open_width_m (float __required_open_width_m))
   (setq _object_geometry __object_geometry)
   (setq _model_choice (string __model_choice))
   (setq _plan_id (string __plan_id))
   (setq _diagnostic (string __diagnostic))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:valid
   (&optional (__valid :null))
   (if (not (eq __valid :null)) (setq _valid __valid)) _valid)
  (:poses
   (&rest __poses)
   (if (keywordp (car __poses))
       (send* _poses __poses)
     (progn
       (if __poses (setq _poses (car __poses)))
       _poses)))
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:candidate_width_m
   (&optional __candidate_width_m)
   (if __candidate_width_m (setq _candidate_width_m __candidate_width_m)) _candidate_width_m)
  (:required_open_width_m
   (&optional __required_open_width_m)
   (if __required_open_width_m (setq _required_open_width_m __required_open_width_m)) _required_open_width_m)
  (:object_geometry
   (&rest __object_geometry)
   (if (keywordp (car __object_geometry))
       (send* _object_geometry __object_geometry)
     (progn
       (if __object_geometry (setq _object_geometry (car __object_geometry)))
       _object_geometry)))
  (:model_choice
   (&optional __model_choice)
   (if __model_choice (setq _model_choice __model_choice)) _model_choice)
  (:plan_id
   (&optional __plan_id)
   (if __plan_id (setq _plan_id __plan_id)) _plan_id)
  (:diagnostic
   (&optional __diagnostic)
   (if __diagnostic (setq _diagnostic __diagnostic)) _diagnostic)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _valid
    1
    ;; geometry_msgs/Pose[] _poses
    (apply #'+ (send-all _poses :serialization-length)) 4
    ;; float32 _score
    4
    ;; float32 _candidate_width_m
    4
    ;; float32 _required_open_width_m
    4
    ;; alicia_flexible_grasp_supervisor/ObjectGeometry _object_geometry
    (send _object_geometry :serialization-length)
    ;; string _model_choice
    4 (length _model_choice)
    ;; string _plan_id
    4 (length _plan_id)
    ;; string _diagnostic
    4 (length _diagnostic)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _valid
       (if _valid (write-byte -1 s) (write-byte 0 s))
     ;; geometry_msgs/Pose[] _poses
     (write-long (length _poses) s)
     (dolist (elem _poses)
       (send elem :serialize s)
       )
     ;; float32 _score
       (sys::poke _score (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _candidate_width_m
       (sys::poke _candidate_width_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _required_open_width_m
       (sys::poke _required_open_width_m (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; alicia_flexible_grasp_supervisor/ObjectGeometry _object_geometry
       (send _object_geometry :serialize s)
     ;; string _model_choice
       (write-long (length _model_choice) s) (princ _model_choice s)
     ;; string _plan_id
       (write-long (length _plan_id) s) (princ _plan_id s)
     ;; string _diagnostic
       (write-long (length _diagnostic) s) (princ _diagnostic s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _valid
     (setq _valid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; geometry_msgs/Pose[] _poses
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poses (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _poses)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32 _score
     (setq _score (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _candidate_width_m
     (setq _candidate_width_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _required_open_width_m
     (setq _required_open_width_m (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; alicia_flexible_grasp_supervisor/ObjectGeometry _object_geometry
     (send _object_geometry :deserialize buf ptr-) (incf ptr- (send _object_geometry :serialization-length))
   ;; string _model_choice
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model_choice (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _plan_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _plan_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _diagnostic
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _diagnostic (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get alicia_flexible_grasp_supervisor::Grasp6DPlan :md5sum-) "a90c722154d42b08b0800cd88d52ff9d")
(setf (get alicia_flexible_grasp_supervisor::Grasp6DPlan :datatype-) "alicia_flexible_grasp_supervisor/Grasp6DPlan")
(setf (get alicia_flexible_grasp_supervisor::Grasp6DPlan :definition-)
      "std_msgs/Header header
bool valid
geometry_msgs/Pose[] poses
float32 score
float32 candidate_width_m
float32 required_open_width_m
alicia_flexible_grasp_supervisor/ObjectGeometry object_geometry
string model_choice
string plan_id
string diagnostic

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

================================================================================
MSG: alicia_flexible_grasp_supervisor/ObjectGeometry
std_msgs/Header header
bool valid
string label
string source_mode
geometry_msgs/Pose pose_base
geometry_msgs/Vector3 size_xyz_m
geometry_msgs/Vector3 support_normal_base
float32 support_offset_m
uint32 valid_depth_points
float32 valid_depth_ratio
float32 depth_mad_m
uint32 fused_frames
float32 support_inlier_ratio
uint32 object_point_count
string failure_reason

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :alicia_flexible_grasp_supervisor/Grasp6DPlan "a90c722154d42b08b0800cd88d52ff9d")


