
(cl:in-package :asdf)

(defsystem "custom_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "VehiclePose" :depends-on ("_package_VehiclePose"))
    (:file "_package_VehiclePose" :depends-on ("_package"))
  ))