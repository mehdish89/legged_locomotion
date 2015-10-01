
(cl:in-package :asdf)

(defsystem "daedalus_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Stats" :depends-on ("_package_Stats"))
    (:file "_package_Stats" :depends-on ("_package"))
    (:file "stats" :depends-on ("_package_stats"))
    (:file "_package_stats" :depends-on ("_package"))
  ))