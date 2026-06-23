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
  :slots (_stopped _message ))

(defmethod alicia_flexible_grasp_supervisor::StopGraspResponse
  (:init
   (&key
    ((:stopped __stopped) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _stopped __stopped)
   (setq _message (string __message))
   self)
  (:stopped
   (&optional (__stopped :null))
   (if (not (eq __stopped :null)) (setq _stopped __stopped)) _stopped)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _stopped
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _stopped
       (if _stopped (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _stopped
     (setq _stopped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::StopGrasp
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::StopGrasp :md5sum-) "82bc3459aaf52f4d9d44d2af0299d950")
(setf (get alicia_flexible_grasp_supervisor::StopGrasp :datatype-) "alicia_flexible_grasp_supervisor/StopGrasp")
(setf (get alicia_flexible_grasp_supervisor::StopGrasp :request) alicia_flexible_grasp_supervisor::StopGraspRequest)
(setf (get alicia_flexible_grasp_supervisor::StopGrasp :response) alicia_flexible_grasp_supervisor::StopGraspResponse)

(defmethod alicia_flexible_grasp_supervisor::StopGraspRequest
  (:response () (instance alicia_flexible_grasp_supervisor::StopGraspResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::StopGraspRequest :md5sum-) "82bc3459aaf52f4d9d44d2af0299d950")
(setf (get alicia_flexible_grasp_supervisor::StopGraspRequest :datatype-) "alicia_flexible_grasp_supervisor/StopGraspRequest")
(setf (get alicia_flexible_grasp_supervisor::StopGraspRequest :definition-)
      "bool emergency
---
bool stopped
string message

")

(setf (get alicia_flexible_grasp_supervisor::StopGraspResponse :md5sum-) "82bc3459aaf52f4d9d44d2af0299d950")
(setf (get alicia_flexible_grasp_supervisor::StopGraspResponse :datatype-) "alicia_flexible_grasp_supervisor/StopGraspResponse")
(setf (get alicia_flexible_grasp_supervisor::StopGraspResponse :definition-)
      "bool emergency
---
bool stopped
string message

")



(provide :alicia_flexible_grasp_supervisor/StopGrasp "82bc3459aaf52f4d9d44d2af0299d950")


