
(cl:in-package :asdf)

(defsystem "alicia_duo_driver-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ArmJointState" :depends-on ("_package_ArmJointState"))
    (:file "_package_ArmJointState" :depends-on ("_package"))
  ))