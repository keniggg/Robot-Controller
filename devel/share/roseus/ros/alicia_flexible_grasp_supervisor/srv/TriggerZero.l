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
  :slots (_skin_id ))

(defmethod alicia_flexible_grasp_supervisor::TriggerZeroRequest
  (:init
   (&key
    ((:skin_id __skin_id) 0)
    )
   (send-super :init)
   (setq _skin_id (round __skin_id))
   self)
  (:skin_id
   (&optional __skin_id)
   (if __skin_id (setq _skin_id __skin_id)) _skin_id)
  (:serialization-length
   ()
   (+
    ;; uint8 _skin_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _skin_id
       (write-byte _skin_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _skin_id
     (setq _skin_id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::TriggerZeroResponse
  :super ros::object
  :slots (_ok _message ))

(defmethod alicia_flexible_grasp_supervisor::TriggerZeroResponse
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

(defclass alicia_flexible_grasp_supervisor::TriggerZero
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::TriggerZero :md5sum-) "f7d6a6f40853bc7ccc545a177690ba20")
(setf (get alicia_flexible_grasp_supervisor::TriggerZero :datatype-) "alicia_flexible_grasp_supervisor/TriggerZero")
(setf (get alicia_flexible_grasp_supervisor::TriggerZero :request) alicia_flexible_grasp_supervisor::TriggerZeroRequest)
(setf (get alicia_flexible_grasp_supervisor::TriggerZero :response) alicia_flexible_grasp_supervisor::TriggerZeroResponse)

(defmethod alicia_flexible_grasp_supervisor::TriggerZeroRequest
  (:response () (instance alicia_flexible_grasp_supervisor::TriggerZeroResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::TriggerZeroRequest :md5sum-) "f7d6a6f40853bc7ccc545a177690ba20")
(setf (get alicia_flexible_grasp_supervisor::TriggerZeroRequest :datatype-) "alicia_flexible_grasp_supervisor/TriggerZeroRequest")
(setf (get alicia_flexible_grasp_supervisor::TriggerZeroRequest :definition-)
      "uint8 skin_id
---
bool ok
string message

")

(setf (get alicia_flexible_grasp_supervisor::TriggerZeroResponse :md5sum-) "f7d6a6f40853bc7ccc545a177690ba20")
(setf (get alicia_flexible_grasp_supervisor::TriggerZeroResponse :datatype-) "alicia_flexible_grasp_supervisor/TriggerZeroResponse")
(setf (get alicia_flexible_grasp_supervisor::TriggerZeroResponse :definition-)
      "uint8 skin_id
---
bool ok
string message

")



(provide :alicia_flexible_grasp_supervisor/TriggerZero "f7d6a6f40853bc7ccc545a177690ba20")


