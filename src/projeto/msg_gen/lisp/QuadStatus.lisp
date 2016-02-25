; Auto-generated. Do not edit!


(cl:in-package projeto-msg)


;//! \htmlinclude QuadStatus.msg.html

(cl:defclass <QuadStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (child_frame_id
    :reader child_frame_id
    :initarg :child_frame_id
    :type cl:string
    :initform "")
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vel
    :reader vel
    :initarg :vel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (theta
    :reader theta
    :initarg :theta
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass QuadStatus (<QuadStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuadStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuadStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name projeto-msg:<QuadStatus> is deprecated: use projeto-msg:QuadStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <QuadStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader projeto-msg:header-val is deprecated.  Use projeto-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'child_frame_id-val :lambda-list '(m))
(cl:defmethod child_frame_id-val ((m <QuadStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader projeto-msg:child_frame_id-val is deprecated.  Use projeto-msg:child_frame_id instead.")
  (child_frame_id m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <QuadStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader projeto-msg:position-val is deprecated.  Use projeto-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <QuadStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader projeto-msg:vel-val is deprecated.  Use projeto-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <QuadStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader projeto-msg:theta-val is deprecated.  Use projeto-msg:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuadStatus>) ostream)
  "Serializes a message object of type '<QuadStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'child_frame_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'child_frame_id))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'theta) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuadStatus>) istream)
  "Deserializes a message object of type '<QuadStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'child_frame_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'child_frame_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'theta) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuadStatus>)))
  "Returns string type for a message object of type '<QuadStatus>"
  "projeto/QuadStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuadStatus)))
  "Returns string type for a message object of type 'QuadStatus"
  "projeto/QuadStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuadStatus>)))
  "Returns md5sum for a message object of type '<QuadStatus>"
  "a44ffee83cb7c8bc90166eaeb034f993")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuadStatus)))
  "Returns md5sum for a message object of type 'QuadStatus"
  "a44ffee83cb7c8bc90166eaeb034f993")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuadStatus>)))
  "Returns full string definition for message of type '<QuadStatus>"
  (cl:format cl:nil "Header header~%string child_frame_id~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 vel~%geometry_msgs/Vector3 theta~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuadStatus)))
  "Returns full string definition for message of type 'QuadStatus"
  (cl:format cl:nil "Header header~%string child_frame_id~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 vel~%geometry_msgs/Vector3 theta~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuadStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'child_frame_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'theta))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuadStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'QuadStatus
    (cl:cons ':header (header msg))
    (cl:cons ':child_frame_id (child_frame_id msg))
    (cl:cons ':position (position msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':theta (theta msg))
))
