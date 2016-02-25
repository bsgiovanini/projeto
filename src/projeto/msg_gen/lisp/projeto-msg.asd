
(cl:in-package :asdf)

(defsystem "projeto-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "QuadStatus" :depends-on ("_package_QuadStatus"))
    (:file "_package_QuadStatus" :depends-on ("_package"))
  ))