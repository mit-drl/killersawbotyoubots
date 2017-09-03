; Auto-generated. Do not edit!


(cl:in-package fleet_control-msg)


;//! \htmlinclude VelTwist.msg.html

(cl:defclass <VelTwist> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (velocity
    :reader velocity
    :initarg :velocity
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass VelTwist (<VelTwist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VelTwist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VelTwist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fleet_control-msg:<VelTwist> is deprecated: use fleet_control-msg:VelTwist instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <VelTwist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-msg:target-val is deprecated.  Use fleet_control-msg:target instead.")
  (target m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <VelTwist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fleet_control-msg:velocity-val is deprecated.  Use fleet_control-msg:velocity instead.")
  (velocity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VelTwist>) ostream)
  "Serializes a message object of type '<VelTwist>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VelTwist>) istream)
  "Deserializes a message object of type '<VelTwist>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VelTwist>)))
  "Returns string type for a message object of type '<VelTwist>"
  "fleet_control/VelTwist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VelTwist)))
  "Returns string type for a message object of type 'VelTwist"
  "fleet_control/VelTwist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VelTwist>)))
  "Returns md5sum for a message object of type '<VelTwist>"
  "95bf03a87f5869f40f57f1412d35d311")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VelTwist)))
  "Returns md5sum for a message object of type 'VelTwist"
  "95bf03a87f5869f40f57f1412d35d311")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VelTwist>)))
  "Returns full string definition for message of type '<VelTwist>"
  (cl:format cl:nil "geometry_msgs/Twist target~%std_msgs/Float64 velocity~%~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VelTwist)))
  "Returns full string definition for message of type 'VelTwist"
  (cl:format cl:nil "geometry_msgs/Twist target~%std_msgs/Float64 velocity~%~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VelTwist>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VelTwist>))
  "Converts a ROS message object to a list"
  (cl:list 'VelTwist
    (cl:cons ':target (target msg))
    (cl:cons ':velocity (velocity msg))
))
