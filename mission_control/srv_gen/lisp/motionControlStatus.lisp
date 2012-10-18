; Auto-generated. Do not edit!


(cl:in-package mission_control-srv)


;//! \htmlinclude motionControlStatus-request.msg.html

(cl:defclass <motionControlStatus-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass motionControlStatus-request (<motionControlStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControlStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControlStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<motionControlStatus-request> is deprecated: use mission_control-srv:motionControlStatus-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControlStatus-request>) ostream)
  "Serializes a message object of type '<motionControlStatus-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControlStatus-request>) istream)
  "Deserializes a message object of type '<motionControlStatus-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControlStatus-request>)))
  "Returns string type for a service object of type '<motionControlStatus-request>"
  "mission_control/motionControlStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControlStatus-request)))
  "Returns string type for a service object of type 'motionControlStatus-request"
  "mission_control/motionControlStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControlStatus-request>)))
  "Returns md5sum for a message object of type '<motionControlStatus-request>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControlStatus-request)))
  "Returns md5sum for a message object of type 'motionControlStatus-request"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControlStatus-request>)))
  "Returns full string definition for message of type '<motionControlStatus-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControlStatus-request)))
  "Returns full string definition for message of type 'motionControlStatus-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControlStatus-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControlStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControlStatus-request
))
;//! \htmlinclude motionControlStatus-response.msg.html

(cl:defclass <motionControlStatus-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass motionControlStatus-response (<motionControlStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControlStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControlStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<motionControlStatus-response> is deprecated: use mission_control-srv:motionControlStatus-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <motionControlStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_control-srv:status-val is deprecated.  Use mission_control-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControlStatus-response>) ostream)
  "Serializes a message object of type '<motionControlStatus-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControlStatus-response>) istream)
  "Deserializes a message object of type '<motionControlStatus-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControlStatus-response>)))
  "Returns string type for a service object of type '<motionControlStatus-response>"
  "mission_control/motionControlStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControlStatus-response)))
  "Returns string type for a service object of type 'motionControlStatus-response"
  "mission_control/motionControlStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControlStatus-response>)))
  "Returns md5sum for a message object of type '<motionControlStatus-response>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControlStatus-response)))
  "Returns md5sum for a message object of type 'motionControlStatus-response"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControlStatus-response>)))
  "Returns full string definition for message of type '<motionControlStatus-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControlStatus-response)))
  "Returns full string definition for message of type 'motionControlStatus-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControlStatus-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControlStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControlStatus-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motionControlStatus)))
  'motionControlStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motionControlStatus)))
  'motionControlStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControlStatus)))
  "Returns string type for a service object of type '<motionControlStatus>"
  "mission_control/motionControlStatus")