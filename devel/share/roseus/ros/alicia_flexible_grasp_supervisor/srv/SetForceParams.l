;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::SetForceParams)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'SetForceParams (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFORCEPARAMS")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFORCEPARAMS"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFORCEPARAMSREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFORCEPARAMSREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFORCEPARAMSRESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFORCEPARAMSRESPONSE"))

(in-package "ROS")





(defclass alicia_flexible_grasp_supervisor::SetForceParamsRequest
  :super ros::object
  :slots (_contact_threshold_mn _target_force_mn _max_force_mn ))

(defmethod alicia_flexible_grasp_supervisor::SetForceParamsRequest
  (:init
   (&key
    ((:contact_threshold_mn __contact_threshold_mn) 0.0)
    ((:target_force_mn __target_force_mn) 0.0)
    ((:max_force_mn __max_force_mn) 0.0)
    )
   (send-super :init)
   (setq _contact_threshold_mn (float __contact_threshold_mn))
   (setq _target_force_mn (float __target_force_mn))
   (setq _max_force_mn (float __max_force_mn))
   self)
  (:contact_threshold_mn
   (&optional __contact_threshold_mn)
   (if __contact_threshold_mn (setq _contact_threshold_mn __contact_threshold_mn)) _contact_threshold_mn)
  (:target_force_mn
   (&optional __target_force_mn)
   (if __target_force_mn (setq _target_force_mn __target_force_mn)) _target_force_mn)
  (:max_force_mn
   (&optional __max_force_mn)
   (if __max_force_mn (setq _max_force_mn __max_force_mn)) _max_force_mn)
  (:serialization-length
   ()
   (+
    ;; float32 _contact_threshold_mn
    4
    ;; float32 _target_force_mn
    4
    ;; float32 _max_force_mn
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _contact_threshold_mn
       (sys::poke _contact_threshold_mn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_force_mn
       (sys::poke _target_force_mn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_force_mn
       (sys::poke _max_force_mn (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _contact_threshold_mn
     (setq _contact_threshold_mn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_force_mn
     (setq _target_force_mn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_force_mn
     (setq _max_force_mn (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetForceParamsResponse
  :super ros::object
  :slots (_success _message ))

(defmethod alicia_flexible_grasp_supervisor::SetForceParamsResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetForceParams
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::SetForceParams :md5sum-) "b8d5117a765e68a9f9eef7b1389ac605")
(setf (get alicia_flexible_grasp_supervisor::SetForceParams :datatype-) "alicia_flexible_grasp_supervisor/SetForceParams")
(setf (get alicia_flexible_grasp_supervisor::SetForceParams :request) alicia_flexible_grasp_supervisor::SetForceParamsRequest)
(setf (get alicia_flexible_grasp_supervisor::SetForceParams :response) alicia_flexible_grasp_supervisor::SetForceParamsResponse)

(defmethod alicia_flexible_grasp_supervisor::SetForceParamsRequest
  (:response () (instance alicia_flexible_grasp_supervisor::SetForceParamsResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::SetForceParamsRequest :md5sum-) "b8d5117a765e68a9f9eef7b1389ac605")
(setf (get alicia_flexible_grasp_supervisor::SetForceParamsRequest :datatype-) "alicia_flexible_grasp_supervisor/SetForceParamsRequest")
(setf (get alicia_flexible_grasp_supervisor::SetForceParamsRequest :definition-)
      "float32 contact_threshold_mn
float32 target_force_mn
float32 max_force_mn
---
bool success
string message

")

(setf (get alicia_flexible_grasp_supervisor::SetForceParamsResponse :md5sum-) "b8d5117a765e68a9f9eef7b1389ac605")
(setf (get alicia_flexible_grasp_supervisor::SetForceParamsResponse :datatype-) "alicia_flexible_grasp_supervisor/SetForceParamsResponse")
(setf (get alicia_flexible_grasp_supervisor::SetForceParamsResponse :definition-)
      "float32 contact_threshold_mn
float32 target_force_mn
float32 max_force_mn
---
bool success
string message

")



(provide :alicia_flexible_grasp_supervisor/SetForceParams "b8d5117a765e68a9f9eef7b1389ac605")


