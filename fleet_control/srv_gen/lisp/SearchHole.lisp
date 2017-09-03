; Auto-generated. Do not edit!


(cl:in-package fleet_control-srv)


;//! \htmlinclude SearchHole-request.msg.html

(cl:defclass <SearchHole-request> (roslisp-msg-protocol:ros-message)
  ((hole_name
    :reader hole_name
    :initarg :hole_name
    :type cl:string
    :initform ""))
)

(cl:defclass SearchHole-request (<SearchHole-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchHole-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchHole-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-srv:<SearchHole-request> is deprecated: use fleet_control-srv:SearchHole-request instead.")))

(cl:ensure-generic-function 'hole_name-val :lambda-list '(m))
(cl:defmethod hole_name-val ((m <SearchHole-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:hole_name-val is deprecated.  Use fleet_control-srv:hole_name instead.")
  (hole_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchHole-request>) ostream)
  "Serializes a message object of type '<SearchHole-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'hole_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'hole_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchHole-request>) istream)
  "Deserializes a message object of type '<SearchHole-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchHole-request>)))
  "Returns string type for a service object of type '<SearchHole-request>"
  "fleet_control/SearchHoleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchHole-request)))
  "Returns string type for a service object of type 'SearchHole-request"
  "fleet_control/SearchHoleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchHole-request>)))
  "Returns md5sum for a message object of type '<SearchHole-request>"
  "325cb0df236f063238dc714aa855e7b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchHole-request)))
  "Returns md5sum for a message object of type 'SearchHole-request"
  "325cb0df236f063238dc714aa855e7b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchHole-request>)))
  "Returns full string definition for message of type '<SearchHole-request>"
  (cl:format cl:nil "string hole_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchHole-request)))
  "Returns full string definition for message of type 'SearchHole-request"
  (cl:format cl:nil "string hole_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchHole-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'hole_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchHole-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchHole-request
    (cl:cons ':hole_name (hole_name msg))
))
;//! \htmlinclude SearchHole-response.msg.html

(cl:defclass <SearchHole-response> (roslisp-msg-protocol:ros-message)
  ((hole
    :reader hole
    :initarg :hole
    :type hole_detection-msg:Hole
    :initform (cl:make-instance 'hole_detection-msg:Hole)))
)

(cl:defclass SearchHole-response (<SearchHole-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SearchHole-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SearchHole-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-srv:<SearchHole-response> is deprecated: use fleet_control-srv:SearchHole-response instead.")))

(cl:ensure-generic-function 'hole-val :lambda-list '(m))
(cl:defmethod hole-val ((m <SearchHole-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:hole-val is deprecated.  Use fleet_control-srv:hole instead.")
  (hole m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SearchHole-response>) ostream)
  "Serializes a message object of type '<SearchHole-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hole) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SearchHole-response>) istream)
  "Deserializes a message object of type '<SearchHole-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hole) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SearchHole-response>)))
  "Returns string type for a service object of type '<SearchHole-response>"
  "fleet_control/SearchHoleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchHole-response)))
  "Returns string type for a service object of type 'SearchHole-response"
  "fleet_control/SearchHoleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SearchHole-response>)))
  "Returns md5sum for a message object of type '<SearchHole-response>"
  "325cb0df236f063238dc714aa855e7b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SearchHole-response)))
  "Returns md5sum for a message object of type 'SearchHole-response"
  "325cb0df236f063238dc714aa855e7b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SearchHole-response>)))
  "Returns full string definition for message of type '<SearchHole-response>"
  (cl:format cl:nil "hole_detection/Hole hole~%~%~%================================================================================~%MSG: hole_detection/Hole~%bool found~%float64 width~%geometry_msgs/Point position~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SearchHole-response)))
  "Returns full string definition for message of type 'SearchHole-response"
  (cl:format cl:nil "hole_detection/Hole hole~%~%~%================================================================================~%MSG: hole_detection/Hole~%bool found~%float64 width~%geometry_msgs/Point position~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SearchHole-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hole))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SearchHole-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SearchHole-response
    (cl:cons ':hole (hole msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SearchHole)))
  'SearchHole-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SearchHole)))
  'SearchHole-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SearchHole)))
  "Returns string type for a service object of type '<SearchHole>"
  "fleet_control/SearchHole")