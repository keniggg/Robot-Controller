;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::StartGrasp)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'StartGrasp (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STARTGRASP")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STARTGRASP"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STARTGRASPREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STARTGRASPREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STARTGRASPRESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::STARTGRASPRESPONSE"))

(in-package "ROS")





(defclass alicia_flexible_grasp_supervisor::StartGraspRequest
  :super ros::object
  :slots (_use_latest_object _object_label ))

(defmethod alicia_flexible_grasp_supervisor::StartGraspRequest
  (:init
   (&key
    ((:use_latest_object __use_latest_object) nil)
    ((:object_label __object_label) "")
    )
   (send-super :init)
   (setq _use_latest_object __use_latest_object)
   (setq _object_label (string __object_label))
   self)
  (:use_latest_object
   (&optional (__use_latest_object :null))
   (if (not (eq __use_latest_object :null)) (setq _use_latest_object __use_latest_object)) _use_latest_object)
  (:object_label
   (&optional __object_label)
   (if __object_label (setq _object_label __object_label)) _object_label)
  (:serialization-length
   ()
   (+
    ;; bool _use_latest_object
    1
    ;; string _object_label
    4 (length _object_label)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _use_latest_object
       (if _use_latest_object (write-byte -1 s) (write-byte 0 s))
     ;; string _object_label
       (write-long (length _object_label) s) (princ _object_label s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _use_latest_object
     (setq _use_latest_object (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _object_label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _object_label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::StartGraspResponse
  :super ros::object
  :slots (_accepted _message ))

(defmethod alicia_flexible_grasp_supervisor::StartGraspResponse
  (:init
   (&key
    ((:accepted __accepted) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _accepted __accepted)
   (setq _message (string __message))
   self)
  (:accepted
   (&optional (__accepted :null))
   (if (not (eq __accepted :null)) (setq _accepted __accepted)) _accepted)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _accepted
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _accepted
       (if _accepted (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _accepted
     (setq _accepted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::StartGrasp
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::StartGrasp :md5sum-) "444b03b3725d049d9ce9ac0a3dfa2c54")
(setf (get alicia_flexible_grasp_supervisor::StartGrasp :datatype-) "alicia_flexible_grasp_supervisor/StartGrasp")
(setf (get alicia_flexible_grasp_supervisor::StartGrasp :request) alicia_flexible_grasp_supervisor::StartGraspRequest)
(setf (get alicia_flexible_grasp_supervisor::StartGrasp :response) alicia_flexible_grasp_supervisor::StartGraspResponse)

(defmethod alicia_flexible_grasp_supervisor::StartGraspRequest
  (:response () (instance alicia_flexible_grasp_supervisor::StartGraspResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::StartGraspRequest :md5sum-) "444b03b3725d049d9ce9ac0a3dfa2c54")
(setf (get alicia_flexible_grasp_supervisor::StartGraspRequest :datatype-) "alicia_flexible_grasp_supervisor/StartGraspRequest")
(setf (get alicia_flexible_grasp_supervisor::StartGraspRequest :definition-)
      "bool use_latest_object
string object_label
---
bool accepted
string message

")

(setf (get alicia_flexible_grasp_supervisor::StartGraspResponse :md5sum-) "444b03b3725d049d9ce9ac0a3dfa2c54")
(setf (get alicia_flexible_grasp_supervisor::StartGraspResponse :datatype-) "alicia_flexible_grasp_supervisor/StartGraspResponse")
(setf (get alicia_flexible_grasp_supervisor::StartGraspResponse :definition-)
      "bool use_latest_object
string object_label
---
bool accepted
string message

")



(provide :alicia_flexible_grasp_supervisor/StartGrasp "444b03b3725d049d9ce9ac0a3dfa2c54")


