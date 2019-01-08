
(cl:in-package :asdf)

(defsystem "husky_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "HuskyStatus" :depends-on ("_package_HuskyStatus"))
    (:file "_package_HuskyStatus" :depends-on ("_package"))
  ))