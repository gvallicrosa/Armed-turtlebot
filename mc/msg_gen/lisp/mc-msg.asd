
(cl:in-package :asdf)

(defsystem "mc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "TurtlebotSensorState" :depends-on ("_package_TurtlebotSensorState"))
    (:file "_package_TurtlebotSensorState" :depends-on ("_package"))
    (:file "belief_msg" :depends-on ("_package_belief_msg"))
    (:file "_package_belief_msg" :depends-on ("_package"))
  ))