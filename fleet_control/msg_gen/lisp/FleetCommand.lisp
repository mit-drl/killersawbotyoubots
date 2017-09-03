; Auto-generated. Do not edit!


(cl:in-package fleet_control-msg)


;//! \htmlinclude FleetCommand.msg.html

(cl:defclass <FleetCommand> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform "")
   (data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass FleetCommand (<FleetCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FleetCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FleetCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-msg:<FleetCommand> is deprecated: use fleet_control-msg:FleetCommand instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <FleetCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-msg:command-val is deprecated.  Use fleet_control-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <FleetCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-msg:data-val is deprecated.  Use fleet_control-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FleetCommand>) ostream)
  "Serializes a message object of type '<FleetCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FleetCommand>) istream)
  "Deserializes a message object of type '<FleetCommand>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FleetCommand>)))
  "Returns string type for a message object of type '<FleetCommand>"
  "fleet_control/FleetCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FleetCommand)))
  "Returns string type for a message object of type 'FleetCommand"
  "fleet_control/FleetCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FleetCommand>)))
  "Returns md5sum for a message object of type '<FleetCommand>"
  "3b0e888571447feef9b2a74d608e4a3a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FleetCommand)))
  "Returns md5sum for a message object of type 'FleetCommand"
  "3b0e888571447feef9b2a74d608e4a3a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FleetCommand>)))
  "Returns full string definition for message of type '<FleetCommand>"
  (cl:format cl:nil "string command~%float64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FleetCommand)))
  "Returns full string definition for message of type 'FleetCommand"
  (cl:format cl:nil "string command~%float64 data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FleetCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FleetCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'FleetCommand
    (cl:cons ':command (command msg))
    (cl:cons ':data (data msg))
))
