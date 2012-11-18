; Auto-generated. Do not edit!


(cl:in-package motionControl-srv)


;//! \htmlinclude motionControl_left-request.msg.html

(cl:defclass <motionControl_left-request> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type cl:float
    :initform 0.0)
   (angular
    :reader angular
    :initarg :angular
    :type cl:float
    :initform 0.0))
)

(cl:defclass motionControl_left-request (<motionControl_left-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControl_left-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControl_left-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motionControl-srv:<motionControl_left-request> is deprecated: use motionControl-srv:motionControl_left-request instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <motionControl_left-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motionControl-srv:linear-val is deprecated.  Use motionControl-srv:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <motionControl_left-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motionControl-srv:angular-val is deprecated.  Use motionControl-srv:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControl_left-request>) ostream)
  "Serializes a message object of type '<motionControl_left-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControl_left-request>) istream)
  "Deserializes a message object of type '<motionControl_left-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControl_left-request>)))
  "Returns string type for a service object of type '<motionControl_left-request>"
  "motionControl/motionControl_leftRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_left-request)))
  "Returns string type for a service object of type 'motionControl_left-request"
  "motionControl/motionControl_leftRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControl_left-request>)))
  "Returns md5sum for a message object of type '<motionControl_left-request>"
  "144a16e4d6b53a0dbadc2e617460a173")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControl_left-request)))
  "Returns md5sum for a message object of type 'motionControl_left-request"
  "144a16e4d6b53a0dbadc2e617460a173")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControl_left-request>)))
  "Returns full string definition for message of type '<motionControl_left-request>"
  (cl:format cl:nil "float64 linear~%float64 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControl_left-request)))
  "Returns full string definition for message of type 'motionControl_left-request"
  (cl:format cl:nil "float64 linear~%float64 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControl_left-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControl_left-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControl_left-request
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
))
;//! \htmlinclude motionControl_left-response.msg.html

(cl:defclass <motionControl_left-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass motionControl_left-response (<motionControl_left-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControl_left-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControl_left-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motionControl-srv:<motionControl_left-response> is deprecated: use motionControl-srv:motionControl_left-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControl_left-response>) ostream)
  "Serializes a message object of type '<motionControl_left-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControl_left-response>) istream)
  "Deserializes a message object of type '<motionControl_left-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControl_left-response>)))
  "Returns string type for a service object of type '<motionControl_left-response>"
  "motionControl/motionControl_leftResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_left-response)))
  "Returns string type for a service object of type 'motionControl_left-response"
  "motionControl/motionControl_leftResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControl_left-response>)))
  "Returns md5sum for a message object of type '<motionControl_left-response>"
  "144a16e4d6b53a0dbadc2e617460a173")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControl_left-response)))
  "Returns md5sum for a message object of type 'motionControl_left-response"
  "144a16e4d6b53a0dbadc2e617460a173")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControl_left-response>)))
  "Returns full string definition for message of type '<motionControl_left-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControl_left-response)))
  "Returns full string definition for message of type 'motionControl_left-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControl_left-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControl_left-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControl_left-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motionControl_left)))
  'motionControl_left-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motionControl_left)))
  'motionControl_left-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_left)))
  "Returns string type for a service object of type '<motionControl_left>"
  "motionControl/motionControl_left")