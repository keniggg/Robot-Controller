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
  :slots (_execute ))

(defmethod alicia_flexible_grasp_supervisor::StartGraspRequest
  (:init
   (&key
    ((:execute __execute) nil)
    )
   (send-super :init)
   (setq _execute __execute)
   self)
  (:execute
   (&optional (__execute :null))
   (if (not (eq __execute :null)) (setq _execute __execute)) _execute)
  (:serialization-length
   ()
   (+
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::StartGraspResponse
  :super ros::object
  :slots (_success _message ))

(defmethod alicia_flexible_grasp_supervisor::StartGraspResponse
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

(defclass alicia_flexible_grasp_supervisor::StartGrasp
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::StartGrasp :md5sum-) "c1e3198b68b143183a952c85cd9f744a")
(setf (get alicia_flexible_grasp_supervisor::StartGrasp :datatype-) "alicia_flexible_grasp_supervisor/StartGrasp")
(setf (get alicia_flexible_grasp_supervisor::StartGrasp :request) alicia_flexible_grasp_supervisor::StartGraspRequest)
(setf (get alicia_flexible_grasp_supervisor::StartGrasp :response) alicia_flexible_grasp_supervisor::StartGraspResponse)

(defmethod alicia_flexible_grasp_supervisor::StartGraspRequest
  (:response () (instance alicia_flexible_grasp_supervisor::StartGraspResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::StartGraspRequest :md5sum-) "c1e3198b68b143183a952c85cd9f744a")
(setf (get alicia_flexible_grasp_supervisor::StartGraspRequest :datatype-) "alicia_flexible_grasp_supervisor/StartGraspRequest")
(setf (get alicia_flexible_grasp_supervisor::StartGraspRequest :definition-)
      "bool execute
---
bool success
string message

")

(setf (get alicia_flexible_grasp_supervisor::StartGraspResponse :md5sum-) "c1e3198b68b143183a952c85cd9f744a")
(setf (get alicia_flexible_grasp_supervisor::StartGraspResponse :datatype-) "alicia_flexible_grasp_supervisor/StartGraspResponse")
(setf (get alicia_flexible_grasp_supervisor::StartGraspResponse :definition-)
      "bool execute
---
bool success
string message

")



(provide :alicia_flexible_grasp_supervisor/StartGrasp "c1e3198b68b143183a952c85cd9f744a")


