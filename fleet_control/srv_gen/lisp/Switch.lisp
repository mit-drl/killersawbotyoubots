; Auto-generated. Do not edit!


(cl:in-package fleet_control-srv)


;//! \htmlinclude Switch-request.msg.html

(cl:defclass <Switch-request> (roslisp-msg-protocol:ros-message)
  ((skin
    :reader skin
    :initarg :skin
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Switch-request (<Switch-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Switch-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Switch-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-srv:<Switch-request> is deprecated: use fleet_control-srv:Switch-request instead.")))

(cl:ensure-generic-function 'skin-val :lambda-list '(m))
(cl:defmethod skin-val ((m <Switch-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:skin-val is deprecated.  Use fleet_control-srv:skin instead.")
  (skin m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Switch-request>) ostream)
  "Serializes a message object of type '<Switch-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'skin) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Switch-request>) istream)
  "Deserializes a message object of type '<Switch-request>"
    (cl:setf (cl:slot-value msg 'skin) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Switch-request>)))
  "Returns string type for a service object of type '<Switch-request>"
  "fleet_control/SwitchRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Switch-request)))
  "Returns string type for a service object of type 'Switch-request"
  "fleet_control/SwitchRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Switch-request>)))
  "Returns md5sum for a message object of type '<Switch-request>"
  "36a4aec6eac5e7cc284bd46232f7ae74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Switch-request)))
  "Returns md5sum for a message object of type 'Switch-request"
  "36a4aec6eac5e7cc284bd46232f7ae74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Switch-request>)))
  "Returns full string definition for message of type '<Switch-request>"
  (cl:format cl:nil "bool skin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Switch-request)))
  "Returns full string definition for message of type 'Switch-request"
  (cl:format cl:nil "bool skin~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Switch-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Switch-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Switch-request
    (cl:cons ':skin (skin msg))
))
;//! \htmlinclude Switch-response.msg.html

(cl:defclass <Switch-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Switch-response (<Switch-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Switch-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Switch-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-srv:<Switch-response> is deprecated: use fleet_control-srv:Switch-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Switch-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:success-val is deprecated.  Use fleet_control-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Switch-response>) ostream)
  "Serializes a message object of type '<Switch-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Switch-response>) istream)
  "Deserializes a message object of type '<Switch-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Switch-response>)))
  "Returns string type for a service object of type '<Switch-response>"
  "fleet_control/SwitchResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Switch-response)))
  "Returns string type for a service object of type 'Switch-response"
  "fleet_control/SwitchResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Switch-response>)))
  "Returns md5sum for a message object of type '<Switch-response>"
  "36a4aec6eac5e7cc284bd46232f7ae74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Switch-response)))
  "Returns md5sum for a message object of type 'Switch-response"
  "36a4aec6eac5e7cc284bd46232f7ae74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Switch-response>)))
  "Returns full string definition for message of type '<Switch-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Switch-response)))
  "Returns full string definition for message of type 'Switch-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Switch-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Switch-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Switch-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Switch)))
  'Switch-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Switch)))
  'Switch-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Switch)))
  "Returns string type for a service object of type '<Switch>"
  "fleet_control/Switch")