
(cl:in-package :asdf)

(defsystem "robot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RobotMsg" :depends-on ("_package_RobotMsg"))
    (:file "_package_RobotMsg" :depends-on ("_package"))
    (:file "Twist2DStamped" :depends-on ("_package_Twist2DStamped"))
    (:file "_package_Twist2DStamped" :depends-on ("_package"))
  ))