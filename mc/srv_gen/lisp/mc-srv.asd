
(cl:in-package :asdf)

(defsystem "mc-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "updateBelief" :depends-on ("_package_updateBelief"))
    (:file "_package_updateBelief" :depends-on ("_package"))
    (:file "vs_graspTarget" :depends-on ("_package_vs_graspTarget"))
    (:file "_package_vs_graspTarget" :depends-on ("_package"))
    (:file "mc_updateBelief" :depends-on ("_package_mc_updateBelief"))
    (:file "_package_mc_updateBelief" :depends-on ("_package"))
    (:file "motionControl_timedMove" :depends-on ("_package_motionControl_timedMove"))
    (:file "_package_motionControl_timedMove" :depends-on ("_package"))
    (:file "motionControl_move" :depends-on ("_package_motionControl_move"))
    (:file "_package_motionControl_move" :depends-on ("_package"))
  ))