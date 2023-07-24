; Auto-generated. Do not edit!


(cl:in-package custom_msgs-msg)


;//! \htmlinclude VehiclePose.msg.html

(cl:defclass <VehiclePose> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass VehiclePose (<VehiclePose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VehiclePose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VehiclePose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msgs-msg:<VehiclePose> is deprecated: use custom_msgs-msg:VehiclePose instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <VehiclePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:x-val is deprecated.  Use custom_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <VehiclePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:y-val is deprecated.  Use custom_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <VehiclePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:velocity-val is deprecated.  Use custom_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <VehiclePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msgs-msg:yaw-val is deprecated.  Use custom_msgs-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VehiclePose>) ostream)
  "Serializes a message object of type '<VehiclePose>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VehiclePose>) istream)
  "Deserializes a message object of type '<VehiclePose>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VehiclePose>)))
  "Returns string type for a message object of type '<VehiclePose>"
  "custom_msgs/VehiclePose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VehiclePose)))
  "Returns string type for a message object of type 'VehiclePose"
  "custom_msgs/VehiclePose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VehiclePose>)))
  "Returns md5sum for a message object of type '<VehiclePose>"
  "65d587d5369b55469396df1e25152717")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VehiclePose)))
  "Returns md5sum for a message object of type 'VehiclePose"
  "65d587d5369b55469396df1e25152717")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VehiclePose>)))
  "Returns full string definition for message of type '<VehiclePose>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 velocity~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VehiclePose)))
  "Returns full string definition for message of type 'VehiclePose"
  (cl:format cl:nil "float32 x~%float32 y~%float32 velocity~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VehiclePose>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VehiclePose>))
  "Converts a ROS message object to a list"
  (cl:list 'VehiclePose
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':yaw (yaw msg))
))
