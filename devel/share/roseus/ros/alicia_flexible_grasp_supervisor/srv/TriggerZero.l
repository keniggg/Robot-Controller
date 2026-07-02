;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::TriggerZero)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'TriggerZero (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TRIGGERZERO")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TRIGGERZERO"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TRIGGERZEROREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TRIGGERZEROREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TRIGGERZERORESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::TRIGGERZERORESPONSE"))

(in-package "ROS")





(defclass alicia_flexible_grasp_supervisor::TriggerZeroRequest
  :super ros::object
  :slots (_trigger ))

(defmethod alicia_flexible_grasp_supervisor::TriggerZeroRequest
  (:init
   (&key
    ((:trigger __trigger) nil)
    )
   (send-super :init)
   (setq _trigger __trigger)
   self)
  (:trigger
   (&optional (__trigger :null))
   (if (not (eq __trigger :null)) (setq _trigger __trigger)) _trigger)
  (:serialization-length
   ()
   (+
    ;; bool _trigger
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _trigger
       (if _trigger (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _trigger
     (setq _trigger (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::TriggerZeroResponse
  :super ros::object
  :slots (_success _message ))

(defmethod alicia_flexible_grasp_supervisor::TriggerZeroResponse
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

(defclass alicia_flexible_grasp_supervisor::TriggerZero
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::TriggerZero :md5sum-) "1531357144714babf83a2def976015d9")
(setf (get alicia_flexible_grasp_supervisor::TriggerZero :datatype-) "alicia_flexible_grasp_supervisor/TriggerZero")
(setf (get alicia_flexible_grasp_supervisor::TriggerZero :request) alicia_flexible_grasp_supervisor::TriggerZeroRequest)
(setf (get alicia_flexible_grasp_supervisor::TriggerZero :response) alicia_flexible_grasp_supervisor::TriggerZeroResponse)

(defmethod alicia_flexible_grasp_supervisor::TriggerZeroRequest
  (:response () (instance alicia_flexible_grasp_supervisor::TriggerZeroResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::TriggerZeroRequest :md5sum-) "1531357144714babf83a2def976015d9")
(setf (get alicia_flexible_grasp_supervisor::TriggerZeroRequest :datatype-) "alicia_flexible_grasp_supervisor/TriggerZeroRequest")
(setf (get alicia_flexible_grasp_supervisor::TriggerZeroRequest :definition-)
      "bool trigger
---
bool success
string message

")

(setf (get alicia_flexible_grasp_supervisor::TriggerZeroResponse :md5sum-) "1531357144714babf83a2def976015d9")
(setf (get alicia_flexible_grasp_supervisor::TriggerZeroResponse :datatype-) "alicia_flexible_grasp_supervisor/TriggerZeroResponse")
(setf (get alicia_flexible_grasp_supervisor::TriggerZeroResponse :definition-)
      "bool trigger
---
bool success
string message

")



(provide :alicia_flexible_grasp_supervisor/TriggerZero "1531357144714babf83a2def976015d9")


