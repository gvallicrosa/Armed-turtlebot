; Auto-generated. Do not edit!


(cl:in-package mc-srv)


;//! \htmlinclude motionControl_timedMove-request.msg.html

(cl:defclass <motionControl_timedMove-request> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type cl:float
    :initform 0.0)
   (angular
    :reader angular
    :initarg :angular
    :type cl:float
    :initform 0.0)
   (duration
    :reader duration
    :initarg :duration
    :type cl:float
    :initform 0.0))
)

(cl:defclass motionControl_timedMove-request (<motionControl_timedMove-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControl_timedMove-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControl_timedMove-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mc-srv:<motionControl_timedMove-request> is deprecated: use mc-srv:motionControl_timedMove-request instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <motionControl_timedMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mc-srv:linear-val is deprecated.  Use mc-srv:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <motionControl_timedMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mc-srv:angular-val is deprecated.  Use mc-srv:angular instead.")
  (angular m))

(cl:ensure-generic-function 'duration-val :lambda-list '(m))
(cl:defmethod duration-val ((m <motionControl_timedMove-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mc-srv:duration-val is deprecated.  Use mc-srv:duration instead.")
  (duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControl_timedMove-request>) ostream)
  "Serializes a message object of type '<motionControl_timedMove-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'duration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControl_timedMove-request>) istream)
  "Deserializes a message object of type '<motionControl_timedMove-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'duration) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControl_timedMove-request>)))
  "Returns string type for a service object of type '<motionControl_timedMove-request>"
  "mc/motionControl_timedMoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_timedMove-request)))
  "Returns string type for a service object of type 'motionControl_timedMove-request"
  "mc/motionControl_timedMoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControl_timedMove-request>)))
  "Returns md5sum for a message object of type '<motionControl_timedMove-request>"
  "a5b60effd8a8dc7915aa4668eca10e0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControl_timedMove-request)))
  "Returns md5sum for a message object of type 'motionControl_timedMove-request"
  "a5b60effd8a8dc7915aa4668eca10e0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControl_timedMove-request>)))
  "Returns full string definition for message of type '<motionControl_timedMove-request>"
  (cl:format cl:nil "float64 linear~%float64 angular~%float64 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControl_timedMove-request)))
  "Returns full string definition for message of type 'motionControl_timedMove-request"
  (cl:format cl:nil "float64 linear~%float64 angular~%float64 duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControl_timedMove-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControl_timedMove-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControl_timedMove-request
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
    (cl:cons ':duration (duration msg))
))
;//! \htmlinclude motionControl_timedMove-response.msg.html

(cl:defclass <motionControl_timedMove-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass motionControl_timedMove-response (<motionControl_timedMove-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControl_timedMove-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControl_timedMove-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mc-srv:<motionControl_timedMove-response> is deprecated: use mc-srv:motionControl_timedMove-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControl_timedMove-response>) ostream)
  "Serializes a message object of type '<motionControl_timedMove-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControl_timedMove-response>) istream)
  "Deserializes a message object of type '<motionControl_timedMove-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControl_timedMove-response>)))
  "Returns string type for a service object of type '<motionControl_timedMove-response>"
  "mc/motionControl_timedMoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_timedMove-response)))
  "Returns string type for a service object of type 'motionControl_timedMove-response"
  "mc/motionControl_timedMoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControl_timedMove-response>)))
  "Returns md5sum for a message object of type '<motionControl_timedMove-response>"
  "a5b60effd8a8dc7915aa4668eca10e0f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControl_timedMove-response)))
  "Returns md5sum for a message object of type 'motionControl_timedMove-response"
  "a5b60effd8a8dc7915aa4668eca10e0f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControl_timedMove-response>)))
  "Returns full string definition for message of type '<motionControl_timedMove-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControl_timedMove-response)))
  "Returns full string definition for message of type 'motionControl_timedMove-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControl_timedMove-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControl_timedMove-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControl_timedMove-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motionControl_timedMove)))
  'motionControl_timedMove-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motionControl_timedMove)))
  'motionControl_timedMove-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_timedMove)))
  "Returns string type for a service object of type '<motionControl_timedMove>"
  "mc/motionControl_timedMove")