
(cl:in-package :asdf)

(defsystem "mc-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "belief_msg" :depends-on ("_package_belief_msg"))
    (:file "_package_belief_msg" :depends-on ("_package"))
  ))