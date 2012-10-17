
(cl:in-package :asdf)

(defsystem "mission_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "generatePlan" :depends-on ("_package_generatePlan"))
    (:file "_package_generatePlan" :depends-on ("_package"))
    (:file "executePlan" :depends-on ("_package_executePlan"))
    (:file "_package_executePlan" :depends-on ("_package"))
    (:file "emergencyStop" :depends-on ("_package_emergencyStop"))
    (:file "_package_emergencyStop" :depends-on ("_package"))
  ))