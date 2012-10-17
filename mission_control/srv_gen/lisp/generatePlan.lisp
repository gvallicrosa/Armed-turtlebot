; Auto-generated. Do not edit!


(cl:in-package mission_control-srv)


;//! \htmlinclude generatePlan-request.msg.html

(cl:defclass <generatePlan-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass generatePlan-request (<generatePlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <generatePlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'generatePlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<generatePlan-request> is deprecated: use mission_control-srv:generatePlan-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <generatePlan-request>) ostream)
  "Serializes a message object of type '<generatePlan-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <generatePlan-request>) istream)
  "Deserializes a message object of type '<generatePlan-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<generatePlan-request>)))
  "Returns string type for a service object of type '<generatePlan-request>"
  "mission_control/generatePlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'generatePlan-request)))
  "Returns string type for a service object of type 'generatePlan-request"
  "mission_control/generatePlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<generatePlan-request>)))
  "Returns md5sum for a message object of type '<generatePlan-request>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'generatePlan-request)))
  "Returns md5sum for a message object of type 'generatePlan-request"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<generatePlan-request>)))
  "Returns full string definition for message of type '<generatePlan-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'generatePlan-request)))
  "Returns full string definition for message of type 'generatePlan-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <generatePlan-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <generatePlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'generatePlan-request
))
;//! \htmlinclude generatePlan-response.msg.html

(cl:defclass <generatePlan-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass generatePlan-response (<generatePlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <generatePlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'generatePlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mission_control-srv:<generatePlan-response> is deprecated: use mission_control-srv:generatePlan-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <generatePlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mission_control-srv:status-val is deprecated.  Use mission_control-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <generatePlan-response>) ostream)
  "Serializes a message object of type '<generatePlan-response>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <generatePlan-response>) istream)
  "Deserializes a message object of type '<generatePlan-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<generatePlan-response>)))
  "Returns string type for a service object of type '<generatePlan-response>"
  "mission_control/generatePlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'generatePlan-response)))
  "Returns string type for a service object of type 'generatePlan-response"
  "mission_control/generatePlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<generatePlan-response>)))
  "Returns md5sum for a message object of type '<generatePlan-response>"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'generatePlan-response)))
  "Returns md5sum for a message object of type 'generatePlan-response"
  "581cc55c12abfc219e446416014f6d0e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<generatePlan-response>)))
  "Returns full string definition for message of type '<generatePlan-response>"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'generatePlan-response)))
  "Returns full string definition for message of type 'generatePlan-response"
  (cl:format cl:nil "int8 status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <generatePlan-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <generatePlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'generatePlan-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'generatePlan)))
  'generatePlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'generatePlan)))
  'generatePlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'generatePlan)))
  "Returns string type for a service object of type '<generatePlan>"
  "mission_control/generatePlan")