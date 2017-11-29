;; Auto-generated. Do not edit!


(when (boundp 'robot_msgs::RobotMsg)
  (if (not (find-package "ROBOT_MSGS"))
    (make-package "ROBOT_MSGS"))
  (shadow 'RobotMsg (find-package "ROBOT_MSGS")))
(unless (find-package "ROBOT_MSGS::ROBOTMSG")
  (make-package "ROBOT_MSGS::ROBOTMSG"))

(in-package "ROS")
;;//! \htmlinclude RobotMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass robot_msgs::RobotMsg
  :super ros::object
  :slots (_header _detection _rho _theta _psi _x _y _angle _collision ))

(defmethod robot_msgs::RobotMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detection __detection) nil)
    ((:rho __rho) 0.0)
    ((:theta __theta) 0.0)
    ((:psi __psi) 0.0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:angle __angle) 0.0)
    ((:collision __collision) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _detection __detection)
   (setq _rho (float __rho))
   (setq _theta (float __theta))
   (setq _psi (float __psi))
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _angle (float __angle))
   (setq _collision __collision)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detection
   (&optional __detection)
   (if __detection (setq _detection __detection)) _detection)
  (:rho
   (&optional __rho)
   (if __rho (setq _rho __rho)) _rho)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:psi
   (&optional __psi)
   (if __psi (setq _psi __psi)) _psi)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:collision
   (&optional __collision)
   (if __collision (setq _collision __collision)) _collision)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _detection
    1
    ;; float32 _rho
    4
    ;; float32 _theta
    4
    ;; float32 _psi
    4
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _angle
    4
    ;; bool _collision
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _detection
       (if _detection (write-byte -1 s) (write-byte 0 s))
     ;; float32 _rho
       (sys::poke _rho (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _theta
       (sys::poke _theta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _psi
       (sys::poke _psi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _collision
       (if _collision (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _detection
     (setq _detection (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _rho
     (setq _rho (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _theta
     (setq _theta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _psi
     (setq _psi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _collision
     (setq _collision (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get robot_msgs::RobotMsg :md5sum-) "a67b7856741f6c2cdc71b30825fcf226")
(setf (get robot_msgs::RobotMsg :datatype-) "robot_msgs/RobotMsg")
(setf (get robot_msgs::RobotMsg :definition-)
      "Header header
bool detection
float32 rho
float32 theta
float32 psi
float32 x
float32 y
float32 angle
bool collision
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :robot_msgs/RobotMsg "a67b7856741f6c2cdc71b30825fcf226")


