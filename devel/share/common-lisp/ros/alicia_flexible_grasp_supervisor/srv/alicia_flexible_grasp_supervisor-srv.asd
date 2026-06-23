
(cl:in-package :asdf)

(defsystem "alicia_flexible_grasp_supervisor-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SetForceParams" :depends-on ("_package_SetForceParams"))
    (:file "_package_SetForceParams" :depends-on ("_package"))
    (:file "SetTargetPose" :depends-on ("_package_SetTargetPose"))
    (:file "_package_SetTargetPose" :depends-on ("_package"))
    (:file "StartGrasp" :depends-on ("_package_StartGrasp"))
    (:file "_package_StartGrasp" :depends-on ("_package"))
    (:file "StopGrasp" :depends-on ("_package_StopGrasp"))
    (:file "_package_StopGrasp" :depends-on ("_package"))
    (:file "TriggerZero" :depends-on ("_package_TriggerZero"))
    (:file "_package_TriggerZero" :depends-on ("_package"))
  ))