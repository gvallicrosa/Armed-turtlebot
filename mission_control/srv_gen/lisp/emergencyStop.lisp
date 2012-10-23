; Auto-generated. Do not edit!


(cl:in-package mission_control-srv)


;//! \htmlinclude emergencyStop-request.msg.html

(cl:defclass <emergencyStop-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass emergencyStop-request (<emergencyStop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <emergencyStop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'emergencyStop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<emergencyStop-request> is deprecated: use mission_control-srv:emergencyStop-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <emergencyStop-request>) ostream)
  "Serializes a message object of type '<emergencyStop-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <emergencyStop-request>) istream)
  "Deserializes a message object of type '<emergencyStop-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<emergencyStop-request>)))
  "Returns string type for a service object of type '<emergencyStop-request>"
  "mission_control/emergencyStopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emergencyStop-request)))
  "Returns string type for a service object of type 'emergencyStop-request"
  "mission_control/emergencyStopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<emergencyStop-request>)))
  "Returns md5sum for a message object of type '<emergencyStop-request>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'emergencyStop-request)))
  "Returns md5sum for a message object of type 'emergencyStop-request"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<emergencyStop-request>)))
  "Returns full string definition for message of type '<emergencyStop-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'emergencyStop-request)))
  "Returns full string definition for message of type 'emergencyStop-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <emergencyStop-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <emergencyStop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'emergencyStop-request
))
;//! \htmlinclude emergencyStop-response.msg.html

(cl:defclass <emergencyStop-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass emergencyStop-response (<emergencyStop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <emergencyStop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'emergencyStop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<emergencyStop-response> is deprecated: use mission_control-srv:emergencyStop-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <emergencyStop-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_control-srv:status-val is deprecated.  Use mission_control-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <emergencyStop-response>) ostream)
  "Serializes a message object of type '<emergencyStop-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <emergencyStop-response>) istream)
  "Deserializes a message object of type '<emergencyStop-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<emergencyStop-response>)))
  "Returns string type for a service object of type '<emergencyStop-response>"
  "mission_control/emergencyStopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emergencyStop-response)))
  "Returns string type for a service object of type 'emergencyStop-response"
  "mission_control/emergencyStopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<emergencyStop-response>)))
  "Returns md5sum for a message object of type '<emergencyStop-response>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'emergencyStop-response)))
  "Returns md5sum for a message object of type 'emergencyStop-response"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<emergencyStop-response>)))
  "Returns full string definition for message of type '<emergencyStop-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'emergencyStop-response)))
  "Returns full string definition for message of type 'emergencyStop-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <emergencyStop-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <emergencyStop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'emergencyStop-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'emergencyStop)))
  'emergencyStop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'emergencyStop)))
  'emergencyStop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emergencyStop)))
  "Returns string type for a service object of type '<emergencyStop>"
  "mission_control/emergencyStop")