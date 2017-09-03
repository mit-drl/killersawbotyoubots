
(cl:in-package :asdf)

(defsystem "fleet_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FleetDebug" :depends-on ("_package_FleetDebug"))
    (:file "_package_FleetDebug" :depends-on ("_package"))
    (:file "VelTwist" :depends-on ("_package_VelTwist"))
    (:file "_package_VelTwist" :depends-on ("_package"))
    (:file "Fleet" :depends-on ("_package_Fleet"))
    (:file "_package_Fleet" :depends-on ("_package"))
    (:file "FleetCommand" :depends-on ("_package_FleetCommand"))
    (:file "_package_FleetCommand" :depends-on ("_package"))
  ))