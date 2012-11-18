; Auto-generated. Do not edit!


(cl:in-package motionControl-srv)


;//! \htmlinclude motionControl_stop-request.msg.html

(cl:defclass <motionControl_stop-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass motionControl_stop-request (<motionControl_stop-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControl_stop-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControl_stop-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motionControl-srv:<motionControl_stop-request> is deprecated: use motionControl-srv:motionControl_stop-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControl_stop-request>) ostream)
  "Serializes a message object of type '<motionControl_stop-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControl_stop-request>) istream)
  "Deserializes a message object of type '<motionControl_stop-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControl_stop-request>)))
  "Returns string type for a service object of type '<motionControl_stop-request>"
  "motionControl/motionControl_stopRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_stop-request)))
  "Returns string type for a service object of type 'motionControl_stop-request"
  "motionControl/motionControl_stopRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControl_stop-request>)))
  "Returns md5sum for a message object of type '<motionControl_stop-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControl_stop-request)))
  "Returns md5sum for a message object of type 'motionControl_stop-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControl_stop-request>)))
  "Returns full string definition for message of type '<motionControl_stop-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControl_stop-request)))
  "Returns full string definition for message of type 'motionControl_stop-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControl_stop-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControl_stop-request>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControl_stop-request
))
;//! \htmlinclude motionControl_stop-response.msg.html

(cl:defclass <motionControl_stop-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass motionControl_stop-response (<motionControl_stop-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motionControl_stop-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motionControl_stop-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motionControl-srv:<motionControl_stop-response> is deprecated: use motionControl-srv:motionControl_stop-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motionControl_stop-response>) ostream)
  "Serializes a message object of type '<motionControl_stop-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motionControl_stop-response>) istream)
  "Deserializes a message object of type '<motionControl_stop-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motionControl_stop-response>)))
  "Returns string type for a service object of type '<motionControl_stop-response>"
  "motionControl/motionControl_stopResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_stop-response)))
  "Returns string type for a service object of type 'motionControl_stop-response"
  "motionControl/motionControl_stopResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motionControl_stop-response>)))
  "Returns md5sum for a message object of type '<motionControl_stop-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motionControl_stop-response)))
  "Returns md5sum for a message object of type 'motionControl_stop-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motionControl_stop-response>)))
  "Returns full string definition for message of type '<motionControl_stop-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motionControl_stop-response)))
  "Returns full string definition for message of type 'motionControl_stop-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motionControl_stop-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motionControl_stop-response>))
  "Converts a ROS message object to a list"
  (cl:list 'motionControl_stop-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'motionControl_stop)))
  'motionControl_stop-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'motionControl_stop)))
  'motionControl_stop-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motionControl_stop)))
  "Returns string type for a service object of type '<motionControl_stop>"
  "motionControl/motionControl_stop")