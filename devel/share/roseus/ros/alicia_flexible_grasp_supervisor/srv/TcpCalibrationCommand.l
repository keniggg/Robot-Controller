;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::TcpCalibrationCommand)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'TcpCalibrationCommand (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TCPCALIBRATIONCOMMAND")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TCPCALIBRATIONCOMMAND"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TCPCALIBRATIONCOMMANDREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TCPCALIBRATIONCOMMANDREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TCPCALIBRATIONCOMMANDRESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TCPCALIBRATIONCOMMANDRESPONSE"))

(in-package "ROS")



(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass alicia_flexible_grasp_supervisor::TcpCalibrationCommandRequest
  :super ros::object
  :slots (_command ))

(defmethod alicia_flexible_grasp_supervisor::TcpCalibrationCommandRequest
  (:init
   (&key
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command (string __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::TcpCalibrationCommandResponse
  :super ros::object
  :slots (_success _message _sample_count _tcp_translation _fixed_point _rms_error_m _max_error_m _orientation_span_deg _result_file ))

(defmethod alicia_flexible_grasp_supervisor::TcpCalibrationCommandResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    ((:sample_count __sample_count) 0)
    ((:tcp_translation __tcp_translation) (instance geometry_msgs::Vector3 :init))
    ((:fixed_point __fixed_point) (instance geometry_msgs::Vector3 :init))
    ((:rms_error_m __rms_error_m) 0.0)
    ((:max_error_m __max_error_m) 0.0)
    ((:orientation_span_deg __orientation_span_deg) 0.0)
    ((:result_file __result_file) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   (setq _sample_count (round __sample_count))
   (setq _tcp_translation __tcp_translation)
   (setq _fixed_point __fixed_point)
   (setq _rms_error_m (float __rms_error_m))
   (setq _max_error_m (float __max_error_m))
   (setq _orientation_span_deg (float __orientation_span_deg))
   (setq _result_file (string __result_file))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:sample_count
   (&optional __sample_count)
   (if __sample_count (setq _sample_count __sample_count)) _sample_count)
  (:tcp_translation
   (&rest __tcp_translation)
   (if (keywordp (car __tcp_translation))
       (send* _tcp_translation __tcp_translation)
     (progn
       (if __tcp_translation (setq _tcp_translation (car __tcp_translation)))
       _tcp_translation)))
  (:fixed_point
   (&rest __fixed_point)
   (if (keywordp (car __fixed_point))
       (send* _fixed_point __fixed_point)
     (progn
       (if __fixed_point (setq _fixed_point (car __fixed_point)))
       _fixed_point)))
  (:rms_error_m
   (&optional __rms_error_m)
   (if __rms_error_m (setq _rms_error_m __rms_error_m)) _rms_error_m)
  (:max_error_m
   (&optional __max_error_m)
   (if __max_error_m (setq _max_error_m __max_error_m)) _max_error_m)
  (:orientation_span_deg
   (&optional __orientation_span_deg)
   (if __orientation_span_deg (setq _orientation_span_deg __orientation_span_deg)) _orientation_span_deg)
  (:result_file
   (&optional __result_file)
   (if __result_file (setq _result_file __result_file)) _result_file)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ;; uint32 _sample_count
    4
    ;; geometry_msgs/Vector3 _tcp_translation
    (send _tcp_translation :serialization-length)
    ;; geometry_msgs/Vector3 _fixed_point
    (send _fixed_point :serialization-length)
    ;; float64 _rms_error_m
    8
    ;; float64 _max_error_m
    8
    ;; float64 _orientation_span_deg
    8
    ;; string _result_file
    4 (length _result_file)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;; uint32 _sample_count
       (write-long _sample_count s)
     ;; geometry_msgs/Vector3 _tcp_translation
       (send _tcp_translation :serialize s)
     ;; geometry_msgs/Vector3 _fixed_point
       (send _fixed_point :serialize s)
     ;; float64 _rms_error_m
       (sys::poke _rms_error_m (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _max_error_m
       (sys::poke _max_error_m (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _orientation_span_deg
       (sys::poke _orientation_span_deg (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _result_file
       (write-long (length _result_file) s) (princ _result_file s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _sample_count
     (setq _sample_count (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _tcp_translation
     (send _tcp_translation :deserialize buf ptr-) (incf ptr- (send _tcp_translation :serialization-length))
   ;; geometry_msgs/Vector3 _fixed_point
     (send _fixed_point :deserialize buf ptr-) (incf ptr- (send _fixed_point :serialization-length))
   ;; float64 _rms_error_m
     (setq _rms_error_m (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _max_error_m
     (setq _max_error_m (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _orientation_span_deg
     (setq _orientation_span_deg (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _result_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::TcpCalibrationCommand
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommand :md5sum-) "58e49899f0eda63c395f3d3908079771")
(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommand :datatype-) "alicia_flexible_grasp_supervisor/TcpCalibrationCommand")
(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommand :request) alicia_flexible_grasp_supervisor::TcpCalibrationCommandRequest)
(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommand :response) alicia_flexible_grasp_supervisor::TcpCalibrationCommandResponse)

(defmethod alicia_flexible_grasp_supervisor::TcpCalibrationCommandRequest
  (:response () (instance alicia_flexible_grasp_supervisor::TcpCalibrationCommandResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommandRequest :md5sum-) "58e49899f0eda63c395f3d3908079771")
(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommandRequest :datatype-) "alicia_flexible_grasp_supervisor/TcpCalibrationCommandRequest")
(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommandRequest :definition-)
      "string command
---
bool success
string message
uint32 sample_count
geometry_msgs/Vector3 tcp_translation
geometry_msgs/Vector3 fixed_point
float64 rms_error_m
float64 max_error_m
float64 orientation_span_deg
string result_file


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
")

(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommandResponse :md5sum-) "58e49899f0eda63c395f3d3908079771")
(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommandResponse :datatype-) "alicia_flexible_grasp_supervisor/TcpCalibrationCommandResponse")
(setf (get alicia_flexible_grasp_supervisor::TcpCalibrationCommandResponse :definition-)
      "string command
---
bool success
string message
uint32 sample_count
geometry_msgs/Vector3 tcp_translation
geometry_msgs/Vector3 fixed_point
float64 rms_error_m
float64 max_error_m
float64 orientation_span_deg
string result_file


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
")



(provide :alicia_flexible_grasp_supervisor/TcpCalibrationCommand "58e49899f0eda63c395f3d3908079771")


