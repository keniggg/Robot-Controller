;; Auto-generated. Do not edit!


(when (boundp 'alicia_flexible_grasp_supervisor::CartesianJog)
  (if (not (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
    (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR"))
  (shadow 'CartesianJog (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR")))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::CARTESIANJOG")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::CARTESIANJOG"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::CARTESIANJOGREQUEST")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::CARTESIANJOGREQUEST"))
(unless (find-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::CARTESIANJOGRESPONSE")
  (make-package "ALICIA_FLEXIBLE_GRASP_SUPERVISOR::CARTESIANJOGRESPONSE"))

(in-package "ROS")





(defclass alicia_flexible_grasp_supervisor::CartesianJogRequest
  :super ros::object
  :slots (_dx _dy _dz _droll _dpitch _dyaw _execute ))

(defmethod alicia_flexible_grasp_supervisor::CartesianJogRequest
  (:init
   (&key
    ((:dx __dx) 0.0)
    ((:dy __dy) 0.0)
    ((:dz __dz) 0.0)
    ((:droll __droll) 0.0)
    ((:dpitch __dpitch) 0.0)
    ((:dyaw __dyaw) 0.0)
    ((:execute __execute) nil)
    )
   (send-super :init)
   (setq _dx (float __dx))
   (setq _dy (float __dy))
   (setq _dz (float __dz))
   (setq _droll (float __droll))
   (setq _dpitch (float __dpitch))
   (setq _dyaw (float __dyaw))
   (setq _execute __execute)
   self)
  (:dx
   (&optional __dx)
   (if __dx (setq _dx __dx)) _dx)
  (:dy
   (&optional __dy)
   (if __dy (setq _dy __dy)) _dy)
  (:dz
   (&optional __dz)
   (if __dz (setq _dz __dz)) _dz)
  (:droll
   (&optional __droll)
   (if __droll (setq _droll __droll)) _droll)
  (:dpitch
   (&optional __dpitch)
   (if __dpitch (setq _dpitch __dpitch)) _dpitch)
  (:dyaw
   (&optional __dyaw)
   (if __dyaw (setq _dyaw __dyaw)) _dyaw)
  (:execute
   (&optional (__execute :null))
   (if (not (eq __execute :null)) (setq _execute __execute)) _execute)
  (:serialization-length
   ()
   (+
    ;; float64 _dx
    8
    ;; float64 _dy
    8
    ;; float64 _dz
    8
    ;; float64 _droll
    8
    ;; float64 _dpitch
    8
    ;; float64 _dyaw
    8
    ;; bool _execute
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _dx
       (sys::poke _dx (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dy
       (sys::poke _dy (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dz
       (sys::poke _dz (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _droll
       (sys::poke _droll (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dpitch
       (sys::poke _dpitch (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _dyaw
       (sys::poke _dyaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _execute
       (if _execute (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _dx
     (setq _dx (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dy
     (setq _dy (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dz
     (setq _dz (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _droll
     (setq _droll (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dpitch
     (setq _dpitch (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _dyaw
     (setq _dyaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _execute
     (setq _execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass alicia_flexible_grasp_supervisor::CartesianJogResponse
  :super ros::object
  :slots (_success _message ))

(defmethod alicia_flexible_grasp_supervisor::CartesianJogResponse
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

(defclass alicia_flexible_grasp_supervisor::CartesianJog
  :super ros::object
  :slots ())

(setf (get alicia_flexible_grasp_supervisor::CartesianJog :md5sum-) "fe30d6dd3aeb27b001869b9c8844bee4")
(setf (get alicia_flexible_grasp_supervisor::CartesianJog :datatype-) "alicia_flexible_grasp_supervisor/CartesianJog")
(setf (get alicia_flexible_grasp_supervisor::CartesianJog :request) alicia_flexible_grasp_supervisor::CartesianJogRequest)
(setf (get alicia_flexible_grasp_supervisor::CartesianJog :response) alicia_flexible_grasp_supervisor::CartesianJogResponse)

(defmethod alicia_flexible_grasp_supervisor::CartesianJogRequest
  (:response () (instance alicia_flexible_grasp_supervisor::CartesianJogResponse :init)))

(setf (get alicia_flexible_grasp_supervisor::CartesianJogRequest :md5sum-) "fe30d6dd3aeb27b001869b9c8844bee4")
(setf (get alicia_flexible_grasp_supervisor::CartesianJogRequest :datatype-) "alicia_flexible_grasp_supervisor/CartesianJogRequest")
(setf (get alicia_flexible_grasp_supervisor::CartesianJogRequest :definition-)
      "float64 dx
float64 dy
float64 dz
float64 droll
float64 dpitch
float64 dyaw
bool execute
---
bool success
string message

")

(setf (get alicia_flexible_grasp_supervisor::CartesianJogResponse :md5sum-) "fe30d6dd3aeb27b001869b9c8844bee4")
(setf (get alicia_flexible_grasp_supervisor::CartesianJogResponse :datatype-) "alicia_flexible_grasp_supervisor/CartesianJogResponse")
(setf (get alicia_flexible_grasp_supervisor::CartesianJogResponse :definition-)
      "float64 dx
float64 dy
float64 dz
float64 droll
float64 dpitch
float64 dyaw
bool execute
---
bool success
string message

")



(provide :alicia_flexible_grasp_supervisor/CartesianJog "fe30d6dd3aeb27b001869b9c8844bee4")


