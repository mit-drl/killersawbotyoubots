; Auto-generated. Do not edit!


(cl:in-package fleet_control-srv)


;//! \htmlinclude CheckSuccess-request.msg.html

(cl:defclass <CheckSuccess-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CheckSuccess-request (<CheckSuccess-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckSuccess-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckSuccess-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-srv:<CheckSuccess-request> is deprecated: use fleet_control-srv:CheckSuccess-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckSuccess-request>) ostream)
  "Serializes a message object of type '<CheckSuccess-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckSuccess-request>) istream)
  "Deserializes a message object of type '<CheckSuccess-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckSuccess-request>)))
  "Returns string type for a service object of type '<CheckSuccess-request>"
  "fleet_control/CheckSuccessRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckSuccess-request)))
  "Returns string type for a service object of type 'CheckSuccess-request"
  "fleet_control/CheckSuccessRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckSuccess-request>)))
  "Returns md5sum for a message object of type '<CheckSuccess-request>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckSuccess-request)))
  "Returns md5sum for a message object of type 'CheckSuccess-request"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckSuccess-request>)))
  "Returns full string definition for message of type '<CheckSuccess-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckSuccess-request)))
  "Returns full string definition for message of type 'CheckSuccess-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckSuccess-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckSuccess-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckSuccess-request
))
;//! \htmlinclude CheckSuccess-response.msg.html

(cl:defclass <CheckSuccess-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CheckSuccess-response (<CheckSuccess-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CheckSuccess-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CheckSuccess-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-srv:<CheckSuccess-response> is deprecated: use fleet_control-srv:CheckSuccess-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <CheckSuccess-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-srv:success-val is deprecated.  Use fleet_control-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CheckSuccess-response>) ostream)
  "Serializes a message object of type '<CheckSuccess-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CheckSuccess-response>) istream)
  "Deserializes a message object of type '<CheckSuccess-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CheckSuccess-response>)))
  "Returns string type for a service object of type '<CheckSuccess-response>"
  "fleet_control/CheckSuccessResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckSuccess-response)))
  "Returns string type for a service object of type 'CheckSuccess-response"
  "fleet_control/CheckSuccessResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CheckSuccess-response>)))
  "Returns md5sum for a message object of type '<CheckSuccess-response>"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CheckSuccess-response)))
  "Returns md5sum for a message object of type 'CheckSuccess-response"
  "358e233cde0c8a8bcfea4ce193f8fc15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CheckSuccess-response>)))
  "Returns full string definition for message of type '<CheckSuccess-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CheckSuccess-response)))
  "Returns full string definition for message of type 'CheckSuccess-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CheckSuccess-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CheckSuccess-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CheckSuccess-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CheckSuccess)))
  'CheckSuccess-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CheckSuccess)))
  'CheckSuccess-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CheckSuccess)))
  "Returns string type for a service object of type '<CheckSuccess>"
  "fleet_control/CheckSuccess")