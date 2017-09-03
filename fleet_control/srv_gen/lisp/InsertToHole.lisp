; Auto-generated. Do not edit!


(cl:in-package fleet_control-srv)


;//! \htmlinclude InsertToHole-request.msg.html

(cl:defclass <InsertToHole-request> (roslisp-msg-protocol:ros-message)
  ((hole
    :reader hole
    :initarg :hole
    :type hole_detection-msg:Hole
    :initform (cl:make-instance 'hole_detection-msg:Hole))
   (angle
    :reader angle
    :initarg :angle
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (hole_name
    :reader hole_name
    :initarg :hole_name
    :type cl:string
    :initform ""))
)

(cl:defclass InsertToHole-request (<InsertToHole-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InsertToHole-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InsertToHole-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-srv:<InsertToHole-request> is deprecated: use fleet_control-srv:InsertToHole-request instead.")))

(cl:ensure-generic-function 'hole-val :lambda-list '(m))
(cl:defmethod hole-val ((m <InsertToHole-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:hole-val is deprecated.  Use fleet_control-srv:hole instead.")
  (hole m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <InsertToHole-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:angle-val is deprecated.  Use fleet_control-srv:angle instead.")
  (angle m))

(cl:ensure-generic-function 'hole_name-val :lambda-list '(m))
(cl:defmethod hole_name-val ((m <InsertToHole-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:hole_name-val is deprecated.  Use fleet_control-srv:hole_name instead.")
  (hole_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InsertToHole-request>) ostream)
  "Serializes a message object of type '<InsertToHole-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hole) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angle) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hole_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hole_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InsertToHole-request>) istream)
  "Deserializes a message object of type '<InsertToHole-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hole) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angle) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hole_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'hole_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InsertToHole-request>)))
  "Returns string type for a service object of type '<InsertToHole-request>"
  "fleet_control/InsertToHoleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InsertToHole-request)))
  "Returns string type for a service object of type 'InsertToHole-request"
  "fleet_control/InsertToHoleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InsertToHole-request>)))
  "Returns md5sum for a message object of type '<InsertToHole-request>"
  "e79a0046ad15ab2c2f4f4971fdf613aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InsertToHole-request)))
  "Returns md5sum for a message object of type 'InsertToHole-request"
  "e79a0046ad15ab2c2f4f4971fdf613aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InsertToHole-request>)))
  "Returns full string definition for message of type '<InsertToHole-request>"
  (cl:format cl:nil "hole_detection/Hole hole~%std_msgs/Float64 angle~%string hole_name~%~%================================================================================~%MSG: hole_detection/Hole~%bool found~%float64 width~%geometry_msgs/Point position~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InsertToHole-request)))
  "Returns full string definition for message of type 'InsertToHole-request"
  (cl:format cl:nil "hole_detection/Hole hole~%std_msgs/Float64 angle~%string hole_name~%~%================================================================================~%MSG: hole_detection/Hole~%bool found~%float64 width~%geometry_msgs/Point position~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InsertToHole-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hole))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angle))
     4 (cl:length (cl:slot-value msg 'hole_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InsertToHole-request>))
  "Converts a ROS message object to a list"
  (cl:list 'InsertToHole-request
    (cl:cons ':hole (hole msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':hole_name (hole_name msg))
))
;//! \htmlinclude InsertToHole-response.msg.html

(cl:defclass <InsertToHole-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass InsertToHole-response (<InsertToHole-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <InsertToHole-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'InsertToHole-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-srv:<InsertToHole-response> is deprecated: use fleet_control-srv:InsertToHole-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <InsertToHole-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:success-val is deprecated.  Use fleet_control-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <InsertToHole-response>) ostream)
  "Serializes a message object of type '<InsertToHole-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <InsertToHole-response>) istream)
  "Deserializes a message object of type '<InsertToHole-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<InsertToHole-response>)))
  "Returns string type for a service object of type '<InsertToHole-response>"
  "fleet_control/InsertToHoleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InsertToHole-response)))
  "Returns string type for a service object of type 'InsertToHole-response"
  "fleet_control/InsertToHoleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<InsertToHole-response>)))
  "Returns md5sum for a message object of type '<InsertToHole-response>"
  "e79a0046ad15ab2c2f4f4971fdf613aa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'InsertToHole-response)))
  "Returns md5sum for a message object of type 'InsertToHole-response"
  "e79a0046ad15ab2c2f4f4971fdf613aa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<InsertToHole-response>)))
  "Returns full string definition for message of type '<InsertToHole-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'InsertToHole-response)))
  "Returns full string definition for message of type 'InsertToHole-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <InsertToHole-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <InsertToHole-response>))
  "Converts a ROS message object to a list"
  (cl:list 'InsertToHole-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'InsertToHole)))
  'InsertToHole-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'InsertToHole)))
  'InsertToHole-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'InsertToHole)))
  "Returns string type for a service object of type '<InsertToHole>"
  "fleet_control/InsertToHole")