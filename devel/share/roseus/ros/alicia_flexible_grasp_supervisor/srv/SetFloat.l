;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::SetFloat)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'SetFloat (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFLOAT")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFLOAT"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFLOATREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFLOATREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFLOATRESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETFLOATRESPONSE"))

(in-package "ROS")





(defclass alicia_flexible_grasp_supervisor::SetFloatRequest
  :super ros::object
  :slots (_value ))

(defmethod alicia_flexible_grasp_supervisor::SetFloatRequest
  (:init
   (&key
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _value (float __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; float64 _value
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _value
       (sys::poke _value (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _value
     (setq _value (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetFloatResponse
  :super ros::object
  :slots (_success _message ))

(defmethod alicia_flexible_grasp_supervisor::SetFloatResponse
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

(defclass alicia_flexible_grasp_supervisor::SetFloat
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::SetFloat :md5sum-) "5d5778235c8477c5669230b433701e6c")
(setf (get alicia_flexible_grasp_supervisor::SetFloat :datatype-) "alicia_flexible_grasp_supervisor/SetFloat")
(setf (get alicia_flexible_grasp_supervisor::SetFloat :request) alicia_flexible_grasp_supervisor::SetFloatRequest)
(setf (get alicia_flexible_grasp_supervisor::SetFloat :response) alicia_flexible_grasp_supervisor::SetFloatResponse)

(defmethod alicia_flexible_grasp_supervisor::SetFloatRequest
  (:response () (instance alicia_flexible_grasp_supervisor::SetFloatResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::SetFloatRequest :md5sum-) "5d5778235c8477c5669230b433701e6c")
(setf (get alicia_flexible_grasp_supervisor::SetFloatRequest :datatype-) "alicia_flexible_grasp_supervisor/SetFloatRequest")
(setf (get alicia_flexible_grasp_supervisor::SetFloatRequest :definition-)
      "float64 value
---
bool success
string message
")

(setf (get alicia_flexible_grasp_supervisor::SetFloatResponse :md5sum-) "5d5778235c8477c5669230b433701e6c")
(setf (get alicia_flexible_grasp_supervisor::SetFloatResponse :datatype-) "alicia_flexible_grasp_supervisor/SetFloatResponse")
(setf (get alicia_flexible_grasp_supervisor::SetFloatResponse :definition-)
      "float64 value
---
bool success
string message
")



(provide :alicia_flexible_grasp_supervisor/SetFloat "5d5778235c8477c5669230b433701e6c")


