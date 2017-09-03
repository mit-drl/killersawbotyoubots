; Auto-generated. Do not edit!


(cl:in-package fleet_control-msg)


;//! \htmlinclude FleetDebug.msg.html

(cl:defclass <FleetDebug> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass FleetDebug (<FleetDebug>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FleetDebug>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FleetDebug)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-msg:<FleetDebug> is deprecated: use fleet_control-msg:FleetDebug instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <FleetDebug>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-msg:data-val is deprecated.  Use fleet_control-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FleetDebug>) ostream)
  "Serializes a message object of type '<FleetDebug>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FleetDebug>) istream)
  "Deserializes a message object of type '<FleetDebug>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FleetDebug>)))
  "Returns string type for a message object of type '<FleetDebug>"
  "fleet_control/FleetDebug")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FleetDebug)))
  "Returns string type for a message object of type 'FleetDebug"
  "fleet_control/FleetDebug")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FleetDebug>)))
  "Returns md5sum for a message object of type '<FleetDebug>"
  "420cd38b6b071cd49f2970c3e2cee511")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FleetDebug)))
  "Returns md5sum for a message object of type 'FleetDebug"
  "420cd38b6b071cd49f2970c3e2cee511")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FleetDebug>)))
  "Returns full string definition for message of type '<FleetDebug>"
  (cl:format cl:nil "float32[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FleetDebug)))
  "Returns full string definition for message of type 'FleetDebug"
  (cl:format cl:nil "float32[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FleetDebug>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FleetDebug>))
  "Converts a ROS message object to a list"
  (cl:list 'FleetDebug
    (cl:cons ':data (data msg))
))
