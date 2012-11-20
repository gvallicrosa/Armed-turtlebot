; Auto-generated. Do not edit!


(cl:in-package mc-srv)


;//! \htmlinclude vs_graspTarget-request.msg.html

(cl:defclass <vs_graspTarget-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass vs_graspTarget-request (<vs_graspTarget-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vs_graspTarget-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vs_graspTarget-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mc-srv:<vs_graspTarget-request> is deprecated: use mc-srv:vs_graspTarget-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vs_graspTarget-request>) ostream)
  "Serializes a message object of type '<vs_graspTarget-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vs_graspTarget-request>) istream)
  "Deserializes a message object of type '<vs_graspTarget-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vs_graspTarget-request>)))
  "Returns string type for a service object of type '<vs_graspTarget-request>"
  "mc/vs_graspTargetRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vs_graspTarget-request)))
  "Returns string type for a service object of type 'vs_graspTarget-request"
  "mc/vs_graspTargetRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vs_graspTarget-request>)))
  "Returns md5sum for a message object of type '<vs_graspTarget-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vs_graspTarget-request)))
  "Returns md5sum for a message object of type 'vs_graspTarget-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vs_graspTarget-request>)))
  "Returns full string definition for message of type '<vs_graspTarget-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vs_graspTarget-request)))
  "Returns full string definition for message of type 'vs_graspTarget-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vs_graspTarget-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vs_graspTarget-request>))
  "Converts a ROS message object to a list"
  (cl:list 'vs_graspTarget-request
))
;//! \htmlinclude vs_graspTarget-response.msg.html

(cl:defclass <vs_graspTarget-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass vs_graspTarget-response (<vs_graspTarget-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <vs_graspTarget-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'vs_graspTarget-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mc-srv:<vs_graspTarget-response> is deprecated: use mc-srv:vs_graspTarget-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <vs_graspTarget-response>) ostream)
  "Serializes a message object of type '<vs_graspTarget-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <vs_graspTarget-response>) istream)
  "Deserializes a message object of type '<vs_graspTarget-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<vs_graspTarget-response>)))
  "Returns string type for a service object of type '<vs_graspTarget-response>"
  "mc/vs_graspTargetResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vs_graspTarget-response)))
  "Returns string type for a service object of type 'vs_graspTarget-response"
  "mc/vs_graspTargetResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<vs_graspTarget-response>)))
  "Returns md5sum for a message object of type '<vs_graspTarget-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'vs_graspTarget-response)))
  "Returns md5sum for a message object of type 'vs_graspTarget-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<vs_graspTarget-response>)))
  "Returns full string definition for message of type '<vs_graspTarget-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'vs_graspTarget-response)))
  "Returns full string definition for message of type 'vs_graspTarget-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <vs_graspTarget-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <vs_graspTarget-response>))
  "Converts a ROS message object to a list"
  (cl:list 'vs_graspTarget-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'vs_graspTarget)))
  'vs_graspTarget-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'vs_graspTarget)))
  'vs_graspTarget-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'vs_graspTarget)))
  "Returns string type for a service object of type '<vs_graspTarget>"
  "mc/vs_graspTarget")