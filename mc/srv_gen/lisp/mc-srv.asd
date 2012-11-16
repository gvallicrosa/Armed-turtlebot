
(cl:in-package :asdf)

(defsystem "mc-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "updateBelief" :depends-on ("_package_updateBelief"))
    (:file "_package_updateBelief" :depends-on ("_package"))
  ))