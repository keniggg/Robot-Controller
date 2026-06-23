
(cl:in-package :asdf)

(defsystem "alicia_flexible_grasp_supervisor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "GraspState" :depends-on ("_package_GraspState"))
    (:file "_package_GraspState" :depends-on ("_package"))
    (:file "ObjectPose" :depends-on ("_package_ObjectPose"))
    (:file "_package_ObjectPose" :depends-on ("_package"))
    (:file "SafetyState" :depends-on ("_package_SafetyState"))
    (:file "_package_SafetyState" :depends-on ("_package"))
    (:file "TactileFrame" :depends-on ("_package_TactileFrame"))
    (:file "_package_TactileFrame" :depends-on ("_package"))
    (:file "TactileState" :depends-on ("_package_TactileState"))
    (:file "_package_TactileState" :depends-on ("_package"))
  ))