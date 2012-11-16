; Auto-generated. Do not edit!


(cl:in-package mc-srv)


;//! \htmlinclude updateBelief-request.msg.html

(cl:defclass <updateBelief-request> (roslisp-msg-protocol:ros-message)
  ((belief
    :reader belief
    :initarg :belief
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass updateBelief-request (<updateBelief-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <updateBelief-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'updateBelief-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mc-srv:<updateBelief-request> is deprecated: use mc-srv:updateBelief-request instead.")))

(cl:ensure-generic-function 'belief-val :lambda-list '(m))
(cl:defmethod belief-val ((m <updateBelief-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mc-srv:belief-val is deprecated.  Use mc-srv:belief instead.")
  (belief m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <updateBelief-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mc-srv:value-val is deprecated.  Use mc-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <updateBelief-request>) ostream)
  "Serializes a message object of type '<updateBelief-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'belief))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'belief))
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <updateBelief-request>) istream)
  "Deserializes a message object of type '<updateBelief-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'belief) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'belief) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<updateBelief-request>)))
  "Returns string type for a service object of type '<updateBelief-request>"
  "mc/updateBeliefRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'updateBelief-request)))
  "Returns string type for a service object of type 'updateBelief-request"
  "mc/updateBeliefRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<updateBelief-request>)))
  "Returns md5sum for a message object of type '<updateBelief-request>"
  "e50891a9fcb180110e784cd3637d31ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'updateBelief-request)))
  "Returns md5sum for a message object of type 'updateBelief-request"
  "e50891a9fcb180110e784cd3637d31ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<updateBelief-request>)))
  "Returns full string definition for message of type '<updateBelief-request>"
  (cl:format cl:nil "string belief~%int64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'updateBelief-request)))
  "Returns full string definition for message of type 'updateBelief-request"
  (cl:format cl:nil "string belief~%int64 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <updateBelief-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'belief))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <updateBelief-request>))
  "Converts a ROS message object to a list"
  (cl:list 'updateBelief-request
    (cl:cons ':belief (belief msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude updateBelief-response.msg.html

(cl:defclass <updateBelief-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass updateBelief-response (<updateBelief-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <updateBelief-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'updateBelief-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mc-srv:<updateBelief-response> is deprecated: use mc-srv:updateBelief-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <updateBelief-response>) ostream)
  "Serializes a message object of type '<updateBelief-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <updateBelief-response>) istream)
  "Deserializes a message object of type '<updateBelief-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<updateBelief-response>)))
  "Returns string type for a service object of type '<updateBelief-response>"
  "mc/updateBeliefResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'updateBelief-response)))
  "Returns string type for a service object of type 'updateBelief-response"
  "mc/updateBeliefResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<updateBelief-response>)))
  "Returns md5sum for a message object of type '<updateBelief-response>"
  "e50891a9fcb180110e784cd3637d31ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'updateBelief-response)))
  "Returns md5sum for a message object of type 'updateBelief-response"
  "e50891a9fcb180110e784cd3637d31ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<updateBelief-response>)))
  "Returns full string definition for message of type '<updateBelief-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'updateBelief-response)))
  "Returns full string definition for message of type 'updateBelief-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <updateBelief-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <updateBelief-response>))
  "Converts a ROS message object to a list"
  (cl:list 'updateBelief-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'updateBelief)))
  'updateBelief-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'updateBelief)))
  'updateBelief-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'updateBelief)))
  "Returns string type for a service object of type '<updateBelief>"
  "mc/updateBelief")