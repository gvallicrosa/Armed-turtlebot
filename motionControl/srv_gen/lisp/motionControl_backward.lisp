; Auto-generated. Do not edit!


(cl:in-package motionControl-srv)


;//! \htmlinclude motionControl_backward-request.msg.html

(cl:defclass <motionControl_backward-request> (roslisp-msg-protocol:ros-message)
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

(cl:defclass motionControl_backward-request (<motionControl_backward-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControl_backward-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControl_backward-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motionControl-srv:<motionControl_backward-request> is deprecated: use motionControl-srv:motionControl_backward-request instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <motionControl_backward-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motionControl-srv:linear-val is deprecated.  Use motionControl-srv:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <motionControl_backward-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motionControl-srv:angular-val is deprecated.  Use motionControl-srv:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControl_backward-request>) ostream)
  "Serializes a message object of type '<motionControl_backward-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControl_backward-request>) istream)
  "Deserializes a message object of type '<motionControl_backward-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControl_backward-request>)))
  "Returns string type for a service object of type '<motionControl_backward-request>"
  "motionControl/motionControl_backwardRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_backward-request)))
  "Returns string type for a service object of type 'motionControl_backward-request"
  "motionControl/motionControl_backwardRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControl_backward-request>)))
  "Returns md5sum for a message object of type '<motionControl_backward-request>"
  "144a16e4d6b53a0dbadc2e617460a173")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControl_backward-request)))
  "Returns md5sum for a message object of type 'motionControl_backward-request"
  "144a16e4d6b53a0dbadc2e617460a173")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControl_backward-request>)))
  "Returns full string definition for message of type '<motionControl_backward-request>"
  (cl:format cl:nil "float64 linear~%float64 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControl_backward-request)))
  "Returns full string definition for message of type 'motionControl_backward-request"
  (cl:format cl:nil "float64 linear~%float64 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControl_backward-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControl_backward-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControl_backward-request
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
))
;//! \htmlinclude motionControl_backward-response.msg.html

(cl:defclass <motionControl_backward-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass motionControl_backward-response (<motionControl_backward-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControl_backward-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControl_backward-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motionControl-srv:<motionControl_backward-response> is deprecated: use motionControl-srv:motionControl_backward-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControl_backward-response>) ostream)
  "Serializes a message object of type '<motionControl_backward-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControl_backward-response>) istream)
  "Deserializes a message object of type '<motionControl_backward-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControl_backward-response>)))
  "Returns string type for a service object of type '<motionControl_backward-response>"
  "motionControl/motionControl_backwardResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_backward-response)))
  "Returns string type for a service object of type 'motionControl_backward-response"
  "motionControl/motionControl_backwardResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControl_backward-response>)))
  "Returns md5sum for a message object of type '<motionControl_backward-response>"
  "144a16e4d6b53a0dbadc2e617460a173")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControl_backward-response)))
  "Returns md5sum for a message object of type 'motionControl_backward-response"
  "144a16e4d6b53a0dbadc2e617460a173")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControl_backward-response>)))
  "Returns full string definition for message of type '<motionControl_backward-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControl_backward-response)))
  "Returns full string definition for message of type 'motionControl_backward-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControl_backward-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControl_backward-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControl_backward-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motionControl_backward)))
  'motionControl_backward-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motionControl_backward)))
  'motionControl_backward-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_backward)))
  "Returns string type for a service object of type '<motionControl_backward>"
  "motionControl/motionControl_backward")