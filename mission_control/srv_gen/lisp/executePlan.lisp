; Auto-generated. Do not edit!


(cl:in-package mission_control-srv)


;//! \htmlinclude executePlan-request.msg.html

(cl:defclass <executePlan-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass executePlan-request (<executePlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <executePlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'executePlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<executePlan-request> is deprecated: use mission_control-srv:executePlan-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <executePlan-request>) ostream)
  "Serializes a message object of type '<executePlan-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <executePlan-request>) istream)
  "Deserializes a message object of type '<executePlan-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<executePlan-request>)))
  "Returns string type for a service object of type '<executePlan-request>"
  "mission_control/executePlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'executePlan-request)))
  "Returns string type for a service object of type 'executePlan-request"
  "mission_control/executePlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<executePlan-request>)))
  "Returns md5sum for a message object of type '<executePlan-request>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'executePlan-request)))
  "Returns md5sum for a message object of type 'executePlan-request"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<executePlan-request>)))
  "Returns full string definition for message of type '<executePlan-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'executePlan-request)))
  "Returns full string definition for message of type 'executePlan-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <executePlan-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <executePlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'executePlan-request
))
;//! \htmlinclude executePlan-response.msg.html

(cl:defclass <executePlan-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass executePlan-response (<executePlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <executePlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'executePlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<executePlan-response> is deprecated: use mission_control-srv:executePlan-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <executePlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_control-srv:status-val is deprecated.  Use mission_control-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <executePlan-response>) ostream)
  "Serializes a message object of type '<executePlan-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <executePlan-response>) istream)
  "Deserializes a message object of type '<executePlan-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<executePlan-response>)))
  "Returns string type for a service object of type '<executePlan-response>"
  "mission_control/executePlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'executePlan-response)))
  "Returns string type for a service object of type 'executePlan-response"
  "mission_control/executePlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<executePlan-response>)))
  "Returns md5sum for a message object of type '<executePlan-response>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'executePlan-response)))
  "Returns md5sum for a message object of type 'executePlan-response"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<executePlan-response>)))
  "Returns full string definition for message of type '<executePlan-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'executePlan-response)))
  "Returns full string definition for message of type 'executePlan-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <executePlan-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <executePlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'executePlan-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'executePlan)))
  'executePlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'executePlan)))
  'executePlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'executePlan)))
  "Returns string type for a service object of type '<executePlan>"
  "mission_control/executePlan")