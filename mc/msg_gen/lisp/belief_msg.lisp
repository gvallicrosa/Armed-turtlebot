; Auto-generated. Do not edit!


(cl:in-package mc-msg)


;//! \htmlinclude belief_msg.msg.html

(cl:defclass <belief_msg> (roslisp-msg-protocol:ros-message)
  ((belief
    :reader belief
    :initarg :belief
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass belief_msg (<belief_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <belief_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'belief_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mc-msg:<belief_msg> is deprecated: use mc-msg:belief_msg instead.")))

(cl:ensure-generic-function 'belief-val :lambda-list '(m))
(cl:defmethod belief-val ((m <belief_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mc-msg:belief-val is deprecated.  Use mc-msg:belief instead.")
  (belief m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <belief_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mc-msg:value-val is deprecated.  Use mc-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <belief_msg>) ostream)
  "Serializes a message object of type '<belief_msg>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'belief))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'belief))
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <belief_msg>) istream)
  "Deserializes a message object of type '<belief_msg>"
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
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<belief_msg>)))
  "Returns string type for a message object of type '<belief_msg>"
  "mc/belief_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'belief_msg)))
  "Returns string type for a message object of type 'belief_msg"
  "mc/belief_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<belief_msg>)))
  "Returns md5sum for a message object of type '<belief_msg>"
  "a874dd055c20e6e3ee861af67dd756ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'belief_msg)))
  "Returns md5sum for a message object of type 'belief_msg"
  "a874dd055c20e6e3ee861af67dd756ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<belief_msg>)))
  "Returns full string definition for message of type '<belief_msg>"
  (cl:format cl:nil "#The mission control node listens for 'belief' messages.~%#Name of belief:~%#E.g. 'crashed', 'targetLocated' or 'atTarget'.~%string belief~%#Belief value/state:~%# 0: false~%# 1: true~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'belief_msg)))
  "Returns full string definition for message of type 'belief_msg"
  (cl:format cl:nil "#The mission control node listens for 'belief' messages.~%#Name of belief:~%#E.g. 'crashed', 'targetLocated' or 'atTarget'.~%string belief~%#Belief value/state:~%# 0: false~%# 1: true~%int8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <belief_msg>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'belief))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <belief_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'belief_msg
    (cl:cons ':belief (belief msg))
    (cl:cons ':value (value msg))
))
