; Auto-generated. Do not edit!


(cl:in-package msg_check-msg)


;//! \htmlinclude PlotDataMsg.msg.html

(cl:defclass <PlotDataMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (curpos
    :reader curpos
    :initarg :curpos
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (despos
    :reader despos
    :initarg :despos
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (curvel
    :reader curvel
    :initarg :curvel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (desvel
    :reader desvel
    :initarg :desvel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (curacc
    :reader curacc
    :initarg :curacc
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (desacc
    :reader desacc
    :initarg :desacc
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (poserr
    :reader poserr
    :initarg :poserr
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velerr
    :reader velerr
    :initarg :velerr
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (curor
    :reader curor
    :initarg :curor
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (desor
    :reader desor
    :initarg :desor
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (thrust
    :reader thrust
    :initarg :thrust
    :type cl:float
    :initform 0.0)
   (M1_pos_err
    :reader M1_pos_err
    :initarg :M1_pos_err
    :type cl:float
    :initform 0.0)
   (M2_pos_err
    :reader M2_pos_err
    :initarg :M2_pos_err
    :type cl:float
    :initform 0.0)
   (M1_vel_err
    :reader M1_vel_err
    :initarg :M1_vel_err
    :type cl:float
    :initform 0.0)
   (M2_vel_err
    :reader M2_vel_err
    :initarg :M2_vel_err
    :type cl:float
    :initform 0.0))
)

(cl:defclass PlotDataMsg (<PlotDataMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlotDataMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlotDataMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msg_check-msg:<PlotDataMsg> is deprecated: use msg_check-msg:PlotDataMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:header-val is deprecated.  Use msg_check-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'curpos-val :lambda-list '(m))
(cl:defmethod curpos-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:curpos-val is deprecated.  Use msg_check-msg:curpos instead.")
  (curpos m))

(cl:ensure-generic-function 'despos-val :lambda-list '(m))
(cl:defmethod despos-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:despos-val is deprecated.  Use msg_check-msg:despos instead.")
  (despos m))

(cl:ensure-generic-function 'curvel-val :lambda-list '(m))
(cl:defmethod curvel-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:curvel-val is deprecated.  Use msg_check-msg:curvel instead.")
  (curvel m))

(cl:ensure-generic-function 'desvel-val :lambda-list '(m))
(cl:defmethod desvel-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:desvel-val is deprecated.  Use msg_check-msg:desvel instead.")
  (desvel m))

(cl:ensure-generic-function 'curacc-val :lambda-list '(m))
(cl:defmethod curacc-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:curacc-val is deprecated.  Use msg_check-msg:curacc instead.")
  (curacc m))

(cl:ensure-generic-function 'desacc-val :lambda-list '(m))
(cl:defmethod desacc-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:desacc-val is deprecated.  Use msg_check-msg:desacc instead.")
  (desacc m))

(cl:ensure-generic-function 'poserr-val :lambda-list '(m))
(cl:defmethod poserr-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:poserr-val is deprecated.  Use msg_check-msg:poserr instead.")
  (poserr m))

(cl:ensure-generic-function 'velerr-val :lambda-list '(m))
(cl:defmethod velerr-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:velerr-val is deprecated.  Use msg_check-msg:velerr instead.")
  (velerr m))

(cl:ensure-generic-function 'curor-val :lambda-list '(m))
(cl:defmethod curor-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:curor-val is deprecated.  Use msg_check-msg:curor instead.")
  (curor m))

(cl:ensure-generic-function 'desor-val :lambda-list '(m))
(cl:defmethod desor-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:desor-val is deprecated.  Use msg_check-msg:desor instead.")
  (desor m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:thrust-val is deprecated.  Use msg_check-msg:thrust instead.")
  (thrust m))

(cl:ensure-generic-function 'M1_pos_err-val :lambda-list '(m))
(cl:defmethod M1_pos_err-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:M1_pos_err-val is deprecated.  Use msg_check-msg:M1_pos_err instead.")
  (M1_pos_err m))

(cl:ensure-generic-function 'M2_pos_err-val :lambda-list '(m))
(cl:defmethod M2_pos_err-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:M2_pos_err-val is deprecated.  Use msg_check-msg:M2_pos_err instead.")
  (M2_pos_err m))

(cl:ensure-generic-function 'M1_vel_err-val :lambda-list '(m))
(cl:defmethod M1_vel_err-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:M1_vel_err-val is deprecated.  Use msg_check-msg:M1_vel_err instead.")
  (M1_vel_err m))

(cl:ensure-generic-function 'M2_vel_err-val :lambda-list '(m))
(cl:defmethod M2_vel_err-val ((m <PlotDataMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:M2_vel_err-val is deprecated.  Use msg_check-msg:M2_vel_err instead.")
  (M2_vel_err m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlotDataMsg>) ostream)
  "Serializes a message object of type '<PlotDataMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'curpos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'despos) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'curvel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desvel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'curacc) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desacc) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'poserr) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velerr) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'curor) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desor) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'M1_pos_err))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'M2_pos_err))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'M1_vel_err))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'M2_vel_err))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlotDataMsg>) istream)
  "Deserializes a message object of type '<PlotDataMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'curpos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'despos) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'curvel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desvel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'curacc) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desacc) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'poserr) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velerr) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'curor) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desor) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'M1_pos_err) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'M2_pos_err) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'M1_vel_err) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'M2_vel_err) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlotDataMsg>)))
  "Returns string type for a message object of type '<PlotDataMsg>"
  "msg_check/PlotDataMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlotDataMsg)))
  "Returns string type for a message object of type 'PlotDataMsg"
  "msg_check/PlotDataMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlotDataMsg>)))
  "Returns md5sum for a message object of type '<PlotDataMsg>"
  "0d96fc0f4d709ad5a1e1b4d1fe446936")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlotDataMsg)))
  "Returns md5sum for a message object of type 'PlotDataMsg"
  "0d96fc0f4d709ad5a1e1b4d1fe446936")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlotDataMsg>)))
  "Returns full string definition for message of type '<PlotDataMsg>"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Vector3 curpos~%geometry_msgs/Vector3 despos~%~%geometry_msgs/Vector3 curvel~%geometry_msgs/Vector3 desvel~%~%geometry_msgs/Vector3 curacc~%geometry_msgs/Vector3 desacc~%~%geometry_msgs/Vector3 poserr~%geometry_msgs/Vector3 velerr~%~%geometry_msgs/Quaternion curor~%geometry_msgs/Quaternion desor~%~%float64 thrust~%float64 M1_pos_err~%float64 M2_pos_err~%float64 M1_vel_err~%float64 M2_vel_err~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlotDataMsg)))
  "Returns full string definition for message of type 'PlotDataMsg"
  (cl:format cl:nil "Header header~%~%geometry_msgs/Vector3 curpos~%geometry_msgs/Vector3 despos~%~%geometry_msgs/Vector3 curvel~%geometry_msgs/Vector3 desvel~%~%geometry_msgs/Vector3 curacc~%geometry_msgs/Vector3 desacc~%~%geometry_msgs/Vector3 poserr~%geometry_msgs/Vector3 velerr~%~%geometry_msgs/Quaternion curor~%geometry_msgs/Quaternion desor~%~%float64 thrust~%float64 M1_pos_err~%float64 M2_pos_err~%float64 M1_vel_err~%float64 M2_vel_err~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlotDataMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'curpos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'despos))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'curvel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desvel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'curacc))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desacc))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'poserr))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velerr))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'curor))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desor))
     8
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlotDataMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PlotDataMsg
    (cl:cons ':header (header msg))
    (cl:cons ':curpos (curpos msg))
    (cl:cons ':despos (despos msg))
    (cl:cons ':curvel (curvel msg))
    (cl:cons ':desvel (desvel msg))
    (cl:cons ':curacc (curacc msg))
    (cl:cons ':desacc (desacc msg))
    (cl:cons ':poserr (poserr msg))
    (cl:cons ':velerr (velerr msg))
    (cl:cons ':curor (curor msg))
    (cl:cons ':desor (desor msg))
    (cl:cons ':thrust (thrust msg))
    (cl:cons ':M1_pos_err (M1_pos_err msg))
    (cl:cons ':M2_pos_err (M2_pos_err msg))
    (cl:cons ':M1_vel_err (M1_vel_err msg))
    (cl:cons ':M2_vel_err (M2_vel_err msg))
))
