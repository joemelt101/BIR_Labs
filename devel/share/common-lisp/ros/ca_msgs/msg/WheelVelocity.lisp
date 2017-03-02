; Auto-generated. Do not edit!


(cl:in-package ca_msgs-msg)


;//! \htmlinclude WheelVelocity.msg.html

(cl:defclass <WheelVelocity> (roslisp-msg-protocol:ros-message)
  ((velocityLeft
    :reader velocityLeft
    :initarg :velocityLeft
    :type cl:float
    :initform 0.0)
   (velocityRight
    :reader velocityRight
    :initarg :velocityRight
    :type cl:float
    :initform 0.0))
)

(cl:defclass WheelVelocity (<WheelVelocity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WheelVelocity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WheelVelocity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ca_msgs-msg:<WheelVelocity> is deprecated: use ca_msgs-msg:WheelVelocity instead.")))

(cl:ensure-generic-function 'velocityLeft-val :lambda-list '(m))
(cl:defmethod velocityLeft-val ((m <WheelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ca_msgs-msg:velocityLeft-val is deprecated.  Use ca_msgs-msg:velocityLeft instead.")
  (velocityLeft m))

(cl:ensure-generic-function 'velocityRight-val :lambda-list '(m))
(cl:defmethod velocityRight-val ((m <WheelVelocity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ca_msgs-msg:velocityRight-val is deprecated.  Use ca_msgs-msg:velocityRight instead.")
  (velocityRight m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WheelVelocity>) ostream)
  "Serializes a message object of type '<WheelVelocity>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocityLeft))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'velocityRight))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WheelVelocity>) istream)
  "Deserializes a message object of type '<WheelVelocity>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocityLeft) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'velocityRight) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WheelVelocity>)))
  "Returns string type for a message object of type '<WheelVelocity>"
  "ca_msgs/WheelVelocity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WheelVelocity)))
  "Returns string type for a message object of type 'WheelVelocity"
  "ca_msgs/WheelVelocity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WheelVelocity>)))
  "Returns md5sum for a message object of type '<WheelVelocity>"
  "6b7b70e8a4003594801b7cd9759d4202")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WheelVelocity)))
  "Returns md5sum for a message object of type 'WheelVelocity"
  "6b7b70e8a4003594801b7cd9759d4202")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WheelVelocity>)))
  "Returns full string definition for message of type '<WheelVelocity>"
  (cl:format cl:nil "float64 velocityLeft~%float64 velocityRight~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WheelVelocity)))
  "Returns full string definition for message of type 'WheelVelocity"
  (cl:format cl:nil "float64 velocityLeft~%float64 velocityRight~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WheelVelocity>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WheelVelocity>))
  "Converts a ROS message object to a list"
  (cl:list 'WheelVelocity
    (cl:cons ':velocityLeft (velocityLeft msg))
    (cl:cons ':velocityRight (velocityRight msg))
))
