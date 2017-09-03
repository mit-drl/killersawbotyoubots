; Auto-generated. Do not edit!


(cl:in-package fleet_control-msg)


;//! \htmlinclude Fleet.msg.html

(cl:defclass <Fleet> (roslisp-msg-protocol:ros-message)
  ((group
    :reader group
    :initarg :group
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Fleet (<Fleet>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Fleet>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Fleet)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-msg:<Fleet> is deprecated: use fleet_control-msg:Fleet instead.")))

(cl:ensure-generic-function 'group-val :lambda-list '(m))
(cl:defmethod group-val ((m <Fleet>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-msg:group-val is deprecated.  Use fleet_control-msg:group instead.")
  (group m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Fleet>) ostream)
  "Serializes a message object of type '<Fleet>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'group))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'group))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Fleet>) istream)
  "Deserializes a message object of type '<Fleet>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'group) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'group)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Fleet>)))
  "Returns string type for a message object of type '<Fleet>"
  "fleet_control/Fleet")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Fleet)))
  "Returns string type for a message object of type 'Fleet"
  "fleet_control/Fleet")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Fleet>)))
  "Returns md5sum for a message object of type '<Fleet>"
  "876a88f5c49c5bbe139b7da2e1201c49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Fleet)))
  "Returns md5sum for a message object of type 'Fleet"
  "876a88f5c49c5bbe139b7da2e1201c49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Fleet>)))
  "Returns full string definition for message of type '<Fleet>"
  (cl:format cl:nil "string[]    group~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Fleet)))
  "Returns full string definition for message of type 'Fleet"
  (cl:format cl:nil "string[]    group~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Fleet>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'group) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Fleet>))
  "Converts a ROS message object to a list"
  (cl:list 'Fleet
    (cl:cons ':group (group msg))
))
