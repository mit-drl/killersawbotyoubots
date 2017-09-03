
(cl:in-package :asdf)

(defsystem "fleet_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :hole_detection-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "InsertToHole" :depends-on ("_package_InsertToHole"))
    (:file "_package_InsertToHole" :depends-on ("_package"))
    (:file "SearchHole" :depends-on ("_package_SearchHole"))
    (:file "_package_SearchHole" :depends-on ("_package"))
    (:file "Switch" :depends-on ("_package_Switch"))
    (:file "_package_Switch" :depends-on ("_package"))
    (:file "CheckSuccess" :depends-on ("_package_CheckSuccess"))
    (:file "_package_CheckSuccess" :depends-on ("_package"))
  ))