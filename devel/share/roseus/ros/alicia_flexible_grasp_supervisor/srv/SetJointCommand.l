;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::SetJointCommand)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'SetJointCommand (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETJOINTCOMMAND")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETJOINTCOMMAND"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETJOINTCOMMANDREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETJOINTCOMMANDREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETJOINTCOMMANDRESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::SETJOINTCOMMANDRESPONSE"))

(in-package "ROS")





(defclass alicia_flexible_grasp_supervisor::SetJointCommandRequest
  :super ros::object
  :slots (_positions _execute ))

(defmethod alicia_flexible_grasp_supervisor::SetJointCommandRequest
  (:init
   (&key
    ((:positions __positions) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:execute __execute) nil)
    )
   (send-super :init)
   (setq _positions __positions)
   (setq _execute __execute)
   self)
  (:positions
   (&optional __positions)
   (if __positions (setq _positions __positions)) _positions)
  (:execute
   (&optional (__execute :null))
   (if (not (eq __execute :null)) (setq _execute __execute)) _execute)
  (:serialization-length
   ()
   (+
    ;; float64[] _positions
    (* 8    (length _positions)) 4
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _positions
     (write-long (length _positions) s)
     (dotimes (i (length _positions))
       (sys::poke (elt _positions i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _positions
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _positions i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::SetJointCommandResponse
  :super ros::object
  :slots (_success _message ))

(defmethod alicia_flexible_grasp_supervisor::SetJointCommandResponse
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

(defclass alicia_flexible_grasp_supervisor::SetJointCommand
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::SetJointCommand :md5sum-) "b0bf18399157799d65e71152f45a1060")
(setf (get alicia_flexible_grasp_supervisor::SetJointCommand :datatype-) "alicia_flexible_grasp_supervisor/SetJointCommand")
(setf (get alicia_flexible_grasp_supervisor::SetJointCommand :request) alicia_flexible_grasp_supervisor::SetJointCommandRequest)
(setf (get alicia_flexible_grasp_supervisor::SetJointCommand :response) alicia_flexible_grasp_supervisor::SetJointCommandResponse)

(defmethod alicia_flexible_grasp_supervisor::SetJointCommandRequest
  (:response () (instance alicia_flexible_grasp_supervisor::SetJointCommandResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::SetJointCommandRequest :md5sum-) "b0bf18399157799d65e71152f45a1060")
(setf (get alicia_flexible_grasp_supervisor::SetJointCommandRequest :datatype-) "alicia_flexible_grasp_supervisor/SetJointCommandRequest")
(setf (get alicia_flexible_grasp_supervisor::SetJointCommandRequest :definition-)
      "float64[] positions
bool execute
---
bool success
string message

")

(setf (get alicia_flexible_grasp_supervisor::SetJointCommandResponse :md5sum-) "b0bf18399157799d65e71152f45a1060")
(setf (get alicia_flexible_grasp_supervisor::SetJointCommandResponse :datatype-) "alicia_flexible_grasp_supervisor/SetJointCommandResponse")
(setf (get alicia_flexible_grasp_supervisor::SetJointCommandResponse :definition-)
      "float64[] positions
bool execute
---
bool success
string message

")



(provide :alicia_flexible_grasp_supervisor/SetJointCommand "b0bf18399157799d65e71152f45a1060")


