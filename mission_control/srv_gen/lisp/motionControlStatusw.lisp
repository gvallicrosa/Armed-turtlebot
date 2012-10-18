; Auto-generated. Do not edit!


(cl:in-package mission_control-srv)


;//! \htmlinclude motionControlStatusw-request.msg.html

(cl:defclass <motionControlStatusw-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass motionControlStatusw-request (<motionControlStatusw-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControlStatusw-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControlStatusw-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<motionControlStatusw-request> is deprecated: use mission_control-srv:motionControlStatusw-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControlStatusw-request>) ostream)
  "Serializes a message object of type '<motionControlStatusw-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControlStatusw-request>) istream)
  "Deserializes a message object of type '<motionControlStatusw-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControlStatusw-request>)))
  "Returns string type for a service object of type '<motionControlStatusw-request>"
  "mission_control/motionControlStatuswRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControlStatusw-request)))
  "Returns string type for a service object of type 'motionControlStatusw-request"
  "mission_control/motionControlStatuswRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControlStatusw-request>)))
  "Returns md5sum for a message object of type '<motionControlStatusw-request>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControlStatusw-request)))
  "Returns md5sum for a message object of type 'motionControlStatusw-request"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControlStatusw-request>)))
  "Returns full string definition for message of type '<motionControlStatusw-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControlStatusw-request)))
  "Returns full string definition for message of type 'motionControlStatusw-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControlStatusw-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControlStatusw-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControlStatusw-request
))
;//! \htmlinclude motionControlStatusw-response.msg.html

(cl:defclass <motionControlStatusw-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass motionControlStatusw-response (<motionControlStatusw-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControlStatusw-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControlStatusw-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<motionControlStatusw-response> is deprecated: use mission_control-srv:motionControlStatusw-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <motionControlStatusw-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_control-srv:status-val is deprecated.  Use mission_control-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControlStatusw-response>) ostream)
  "Serializes a message object of type '<motionControlStatusw-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControlStatusw-response>) istream)
  "Deserializes a message object of type '<motionControlStatusw-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControlStatusw-response>)))
  "Returns string type for a service object of type '<motionControlStatusw-response>"
  "mission_control/motionControlStatuswResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControlStatusw-response)))
  "Returns string type for a service object of type 'motionControlStatusw-response"
  "mission_control/motionControlStatuswResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControlStatusw-response>)))
  "Returns md5sum for a message object of type '<motionControlStatusw-response>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControlStatusw-response)))
  "Returns md5sum for a message object of type 'motionControlStatusw-response"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControlStatusw-response>)))
  "Returns full string definition for message of type '<motionControlStatusw-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControlStatusw-response)))
  "Returns full string definition for message of type 'motionControlStatusw-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControlStatusw-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControlStatusw-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControlStatusw-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motionControlStatusw)))
  'motionControlStatusw-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motionControlStatusw)))
  'motionControlStatusw-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControlStatusw)))
  "Returns string type for a service object of type '<motionControlStatusw>"
  "mission_control/motionControlStatusw")