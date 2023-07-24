;; Auto-generated. Do not edit!


(when (boundp 'custom_msgs::VehiclePose)
  (if (not (find-package "CUSTOM_MSGS"))
    (make-package "CUSTOM_MSGS"))
  (shadow 'VehiclePose (find-package "CUSTOM_MSGS")))
(unless (find-package "CUSTOM_MSGS::VEHICLEPOSE")
  (make-package "CUSTOM_MSGS::VEHICLEPOSE"))

(in-package "ROS")
;;//! \htmlinclude VehiclePose.msg.html


(defclass custom_msgs::VehiclePose
  :super ros::object
  :slots (_x _y _velocity _yaw ))

(defmethod custom_msgs::VehiclePose
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:velocity __velocity) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _velocity (float __velocity))
   (setq _yaw (float __yaw))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _velocity
    4
    ;; float32 _yaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _velocity
       (sys::poke _velocity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _velocity
     (setq _velocity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msgs::VehiclePose :md5sum-) "65d587d5369b55469396df1e25152717")
(setf (get custom_msgs::VehiclePose :datatype-) "custom_msgs/VehiclePose")
(setf (get custom_msgs::VehiclePose :definition-)
      "float32 x
float32 y
float32 velocity
float32 yaw

")



(provide :custom_msgs/VehiclePose "65d587d5369b55469396df1e25152717")


