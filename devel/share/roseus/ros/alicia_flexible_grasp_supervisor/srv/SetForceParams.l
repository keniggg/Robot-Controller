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
  :slots (_contact_threshold _target_force _max_force ))

(defmethod alicia_flexible_grasp_supervisor::SetForceParamsRequest
  (:init
   (&key
    ((:contact_threshold __contact_threshold) 0.0)
    ((:target_force __target_force) 0.0)
    ((:max_force __max_force) 0.0)
    )
   (send-super :init)
   (setq _contact_threshold (float __contact_threshold))
   (setq _target_force (float __target_force))
   (setq _max_force (float __max_force))
   self)
  (:contact_threshold
   (&optional __contact_threshold)
   (if __contact_threshold (setq _contact_threshold __contact_threshold)) _contact_threshold)
  (:target_force
   (&optional __target_force)
   (if __target_force (setq _target_force __target_force)) _target_force)
  (:max_force
   (&optional __max_force)
   (if __max_force (setq _max_force __max_force)) _max_force)
  (:serialization-length
   ()
   (+
    ;; float32 _contact_threshold
    4
    ;; float32 _target_force
    4
    ;; float32 _max_force
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _contact_threshold
       (sys::poke _contact_threshold (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _target_force
       (sys::poke _target_force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _max_force
       (sys::poke _max_force (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _contact_threshold
     (setq _contact_threshold (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _target_force
     (setq _target_force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _max_force
     (setq _max_force (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetForceParamsResponse
  :super ros::object
  :slots (_ok _message ))

(defmethod alicia_flexible_grasp_supervisor::SetForceParamsResponse
  (:init
   (&key
    ((:ok __ok) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _ok __ok)
   (setq _message (string __message))
   self)
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetForceParams
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::SetForceParams :md5sum-) "5ef41667b487a681d2b7b91f8a1e0b10")
(setf (get alicia_flexible_grasp_supervisor::SetForceParams :datatype-) "alicia_flexible_grasp_supervisor/SetForceParams")
(setf (get alicia_flexible_grasp_supervisor::SetForceParams :request) alicia_flexible_grasp_supervisor::SetForceParamsRequest)
(setf (get alicia_flexible_grasp_supervisor::SetForceParams :response) alicia_flexible_grasp_supervisor::SetForceParamsResponse)

(defmethod alicia_flexible_grasp_supervisor::SetForceParamsRequest
  (:response () (instance alicia_flexible_grasp_supervisor::SetForceParamsResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::SetForceParamsRequest :md5sum-) "5ef41667b487a681d2b7b91f8a1e0b10")
(setf (get alicia_flexible_grasp_supervisor::SetForceParamsRequest :datatype-) "alicia_flexible_grasp_supervisor/SetForceParamsRequest")
(setf (get alicia_flexible_grasp_supervisor::SetForceParamsRequest :definition-)
      "float32 contact_threshold
float32 target_force
float32 max_force
---
bool ok
string message

")

(setf (get alicia_flexible_grasp_supervisor::SetForceParamsResponse :md5sum-) "5ef41667b487a681d2b7b91f8a1e0b10")
(setf (get alicia_flexible_grasp_supervisor::SetForceParamsResponse :datatype-) "alicia_flexible_grasp_supervisor/SetForceParamsResponse")
(setf (get alicia_flexible_grasp_supervisor::SetForceParamsResponse :definition-)
      "float32 contact_threshold
float32 target_force
float32 max_force
---
bool ok
string message

")



(provide :alicia_flexible_grasp_supervisor/SetForceParams "5ef41667b487a681d2b7b91f8a1e0b10")


