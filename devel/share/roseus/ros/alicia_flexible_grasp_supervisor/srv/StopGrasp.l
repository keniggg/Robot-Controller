;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::StopGrasp)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'StopGrasp (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STOPGRASP")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STOPGRASP"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STOPGRASPREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STOPGRASPREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STOPGRASPRESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STOPGRASPRESPONSE"))

(in-package "ROS")





(defclass alicia_flexible_grasp_supervisor::StopGraspRequest
  :super ros::object
  :slots (_emergency ))

(defmethod alicia_flexible_grasp_supervisor::StopGraspRequest
  (:init
   (&key
    ((:emergency __emergency) nil)
    )
   (send-super :init)
   (setq _emergency __emergency)
   self)
  (:emergency
   (&optional (__emergency :null))
   (if (not (eq __emergency :null)) (setq _emergency __emergency)) _emergency)
  (:serialization-length
   ()
   (+
    ;; bool _emergency
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _emergency
       (if _emergency (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _emergency
     (setq _emergency (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::StopGraspResponse
  :super ros::object
  :slots (_success _message ))

(defmethod alicia_flexible_grasp_supervisor::StopGraspResponse
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

(defclass alicia_flexible_grasp_supervisor::StopGrasp
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::StopGrasp :md5sum-) "5e6527b0f131fa5341cb29f70ba6b894")
(setf (get alicia_flexible_grasp_supervisor::StopGrasp :datatype-) "alicia_flexible_grasp_supervisor/StopGrasp")
(setf (get alicia_flexible_grasp_supervisor::StopGrasp :request) alicia_flexible_grasp_supervisor::StopGraspRequest)
(setf (get alicia_flexible_grasp_supervisor::StopGrasp :response) alicia_flexible_grasp_supervisor::StopGraspResponse)

(defmethod alicia_flexible_grasp_supervisor::StopGraspRequest
  (:response () (instance alicia_flexible_grasp_supervisor::StopGraspResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::StopGraspRequest :md5sum-) "5e6527b0f131fa5341cb29f70ba6b894")
(setf (get alicia_flexible_grasp_supervisor::StopGraspRequest :datatype-) "alicia_flexible_grasp_supervisor/StopGraspRequest")
(setf (get alicia_flexible_grasp_supervisor::StopGraspRequest :definition-)
      "bool emergency
---
bool success
string message

")

(setf (get alicia_flexible_grasp_supervisor::StopGraspResponse :md5sum-) "5e6527b0f131fa5341cb29f70ba6b894")
(setf (get alicia_flexible_grasp_supervisor::StopGraspResponse :datatype-) "alicia_flexible_grasp_supervisor/StopGraspResponse")
(setf (get alicia_flexible_grasp_supervisor::StopGraspResponse :definition-)
      "bool emergency
---
bool success
string message

")



(provide :alicia_flexible_grasp_supervisor/StopGrasp "5e6527b0f131fa5341cb29f70ba6b894")


