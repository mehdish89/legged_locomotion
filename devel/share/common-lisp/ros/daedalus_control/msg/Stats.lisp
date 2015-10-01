; Auto-generated. Do not edit!


(cl:in-package daedalus_control-msg)


;//! \htmlinclude Stats.msg.html

(cl:defclass <Stats> (roslisp-msg-protocol:ros-message)
  ((effort
    :reader effort
    :initarg :effort
    :type cl:float
    :initform 0.0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:float
    :initform 0.0)
   (velocity_variation
    :reader velocity_variation
    :initarg :velocity_variation
    :type cl:float
    :initform 0.0)
   (leg_x
    :reader leg_x
    :initarg :leg_x
    :type cl:float
    :initform 0.0)
   (leg_y
    :reader leg_y
    :initarg :leg_y
    :type cl:float
    :initform 0.0)
   (leg_z
    :reader leg_z
    :initarg :leg_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Stats (<Stats>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Stats>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Stats)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name daedalus_control-msg:<Stats> is deprecated: use daedalus_control-msg:Stats instead.")))

(cl:ensure-generic-function 'effort-val :lambda-list '(m))
(cl:defmethod effort-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader daedalus_control-msg:effort-val is deprecated.  Use daedalus_control-msg:effort instead.")
  (effort m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader daedalus_control-msg:velocity-val is deprecated.  Use daedalus_control-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'velocity_variation-val :lambda-list '(m))
(cl:defmethod velocity_variation-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader daedalus_control-msg:velocity_variation-val is deprecated.  Use daedalus_control-msg:velocity_variation instead.")
  (velocity_variation m))

(cl:ensure-generic-function 'leg_x-val :lambda-list '(m))
(cl:defmethod leg_x-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader daedalus_control-msg:leg_x-val is deprecated.  Use daedalus_control-msg:leg_x instead.")
  (leg_x m))

(cl:ensure-generic-function 'leg_y-val :lambda-list '(m))
(cl:defmethod leg_y-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader daedalus_control-msg:leg_y-val is deprecated.  Use daedalus_control-msg:leg_y instead.")
  (leg_y m))

(cl:ensure-generic-function 'leg_z-val :lambda-list '(m))
(cl:defmethod leg_z-val ((m <Stats>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader daedalus_control-msg:leg_z-val is deprecated.  Use daedalus_control-msg:leg_z instead.")
  (leg_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Stats>) ostream)
  "Serializes a message object of type '<Stats>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'effort))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'velocity_variation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'leg_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'leg_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'leg_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Stats>) istream)
  "Deserializes a message object of type '<Stats>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'effort) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'velocity_variation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leg_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leg_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leg_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Stats>)))
  "Returns string type for a message object of type '<Stats>"
  "daedalus_control/Stats")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Stats)))
  "Returns string type for a message object of type 'Stats"
  "daedalus_control/Stats")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Stats>)))
  "Returns md5sum for a message object of type '<Stats>"
  "c1371118692462a07da2dac693adb28c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Stats)))
  "Returns md5sum for a message object of type 'Stats"
  "c1371118692462a07da2dac693adb28c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Stats>)))
  "Returns full string definition for message of type '<Stats>"
  (cl:format cl:nil "float32 effort~%float32 velocity~%float32 velocity_variation~%float32 leg_x~%float32 leg_y~%float32 leg_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Stats)))
  "Returns full string definition for message of type 'Stats"
  (cl:format cl:nil "float32 effort~%float32 velocity~%float32 velocity_variation~%float32 leg_x~%float32 leg_y~%float32 leg_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Stats>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Stats>))
  "Converts a ROS message object to a list"
  (cl:list 'Stats
    (cl:cons ':effort (effort msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':velocity_variation (velocity_variation msg))
    (cl:cons ':leg_x (leg_x msg))
    (cl:cons ':leg_y (leg_y msg))
    (cl:cons ':leg_z (leg_z msg))
))
