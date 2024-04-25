;; Auto-generated. Do not edit!


(when (boundp 'msg_check::PlotDataMsg)
  (if (not (find-package "MSG_CHECK"))
    (make-package "MSG_CHECK"))
  (shadow 'PlotDataMsg (find-package "MSG_CHECK")))
(unless (find-package "MSG_CHECK::PLOTDATAMSG")
  (make-package "MSG_CHECK::PLOTDATAMSG"))

(in-package "ROS")
;;//! \htmlinclude PlotDataMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msg_check::PlotDataMsg
  :super ros::object
  :slots (_header _curpos _despos _curvel _desvel _curacc _desacc _poserr _velerr _curor _desor _thrust _M1_pos_err _M2_pos_err _M1_vel_err _M2_vel_err ))

(defmethod msg_check::PlotDataMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:curpos __curpos) (instance geometry_msgs::Vector3 :init))
    ((:despos __despos) (instance geometry_msgs::Vector3 :init))
    ((:curvel __curvel) (instance geometry_msgs::Vector3 :init))
    ((:desvel __desvel) (instance geometry_msgs::Vector3 :init))
    ((:curacc __curacc) (instance geometry_msgs::Vector3 :init))
    ((:desacc __desacc) (instance geometry_msgs::Vector3 :init))
    ((:poserr __poserr) (instance geometry_msgs::Vector3 :init))
    ((:velerr __velerr) (instance geometry_msgs::Vector3 :init))
    ((:curor __curor) (instance geometry_msgs::Quaternion :init))
    ((:desor __desor) (instance geometry_msgs::Quaternion :init))
    ((:thrust __thrust) 0.0)
    ((:M1_pos_err __M1_pos_err) 0.0)
    ((:M2_pos_err __M2_pos_err) 0.0)
    ((:M1_vel_err __M1_vel_err) 0.0)
    ((:M2_vel_err __M2_vel_err) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _curpos __curpos)
   (setq _despos __despos)
   (setq _curvel __curvel)
   (setq _desvel __desvel)
   (setq _curacc __curacc)
   (setq _desacc __desacc)
   (setq _poserr __poserr)
   (setq _velerr __velerr)
   (setq _curor __curor)
   (setq _desor __desor)
   (setq _thrust (float __thrust))
   (setq _M1_pos_err (float __M1_pos_err))
   (setq _M2_pos_err (float __M2_pos_err))
   (setq _M1_vel_err (float __M1_vel_err))
   (setq _M2_vel_err (float __M2_vel_err))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:curpos
   (&rest __curpos)
   (if (keywordp (car __curpos))
       (send* _curpos __curpos)
     (progn
       (if __curpos (setq _curpos (car __curpos)))
       _curpos)))
  (:despos
   (&rest __despos)
   (if (keywordp (car __despos))
       (send* _despos __despos)
     (progn
       (if __despos (setq _despos (car __despos)))
       _despos)))
  (:curvel
   (&rest __curvel)
   (if (keywordp (car __curvel))
       (send* _curvel __curvel)
     (progn
       (if __curvel (setq _curvel (car __curvel)))
       _curvel)))
  (:desvel
   (&rest __desvel)
   (if (keywordp (car __desvel))
       (send* _desvel __desvel)
     (progn
       (if __desvel (setq _desvel (car __desvel)))
       _desvel)))
  (:curacc
   (&rest __curacc)
   (if (keywordp (car __curacc))
       (send* _curacc __curacc)
     (progn
       (if __curacc (setq _curacc (car __curacc)))
       _curacc)))
  (:desacc
   (&rest __desacc)
   (if (keywordp (car __desacc))
       (send* _desacc __desacc)
     (progn
       (if __desacc (setq _desacc (car __desacc)))
       _desacc)))
  (:poserr
   (&rest __poserr)
   (if (keywordp (car __poserr))
       (send* _poserr __poserr)
     (progn
       (if __poserr (setq _poserr (car __poserr)))
       _poserr)))
  (:velerr
   (&rest __velerr)
   (if (keywordp (car __velerr))
       (send* _velerr __velerr)
     (progn
       (if __velerr (setq _velerr (car __velerr)))
       _velerr)))
  (:curor
   (&rest __curor)
   (if (keywordp (car __curor))
       (send* _curor __curor)
     (progn
       (if __curor (setq _curor (car __curor)))
       _curor)))
  (:desor
   (&rest __desor)
   (if (keywordp (car __desor))
       (send* _desor __desor)
     (progn
       (if __desor (setq _desor (car __desor)))
       _desor)))
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:M1_pos_err
   (&optional __M1_pos_err)
   (if __M1_pos_err (setq _M1_pos_err __M1_pos_err)) _M1_pos_err)
  (:M2_pos_err
   (&optional __M2_pos_err)
   (if __M2_pos_err (setq _M2_pos_err __M2_pos_err)) _M2_pos_err)
  (:M1_vel_err
   (&optional __M1_vel_err)
   (if __M1_vel_err (setq _M1_vel_err __M1_vel_err)) _M1_vel_err)
  (:M2_vel_err
   (&optional __M2_vel_err)
   (if __M2_vel_err (setq _M2_vel_err __M2_vel_err)) _M2_vel_err)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _curpos
    (send _curpos :serialization-length)
    ;; geometry_msgs/Vector3 _despos
    (send _despos :serialization-length)
    ;; geometry_msgs/Vector3 _curvel
    (send _curvel :serialization-length)
    ;; geometry_msgs/Vector3 _desvel
    (send _desvel :serialization-length)
    ;; geometry_msgs/Vector3 _curacc
    (send _curacc :serialization-length)
    ;; geometry_msgs/Vector3 _desacc
    (send _desacc :serialization-length)
    ;; geometry_msgs/Vector3 _poserr
    (send _poserr :serialization-length)
    ;; geometry_msgs/Vector3 _velerr
    (send _velerr :serialization-length)
    ;; geometry_msgs/Quaternion _curor
    (send _curor :serialization-length)
    ;; geometry_msgs/Quaternion _desor
    (send _desor :serialization-length)
    ;; float64 _thrust
    8
    ;; float64 _M1_pos_err
    8
    ;; float64 _M2_pos_err
    8
    ;; float64 _M1_vel_err
    8
    ;; float64 _M2_vel_err
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _curpos
       (send _curpos :serialize s)
     ;; geometry_msgs/Vector3 _despos
       (send _despos :serialize s)
     ;; geometry_msgs/Vector3 _curvel
       (send _curvel :serialize s)
     ;; geometry_msgs/Vector3 _desvel
       (send _desvel :serialize s)
     ;; geometry_msgs/Vector3 _curacc
       (send _curacc :serialize s)
     ;; geometry_msgs/Vector3 _desacc
       (send _desacc :serialize s)
     ;; geometry_msgs/Vector3 _poserr
       (send _poserr :serialize s)
     ;; geometry_msgs/Vector3 _velerr
       (send _velerr :serialize s)
     ;; geometry_msgs/Quaternion _curor
       (send _curor :serialize s)
     ;; geometry_msgs/Quaternion _desor
       (send _desor :serialize s)
     ;; float64 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _M1_pos_err
       (sys::poke _M1_pos_err (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _M2_pos_err
       (sys::poke _M2_pos_err (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _M1_vel_err
       (sys::poke _M1_vel_err (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _M2_vel_err
       (sys::poke _M2_vel_err (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _curpos
     (send _curpos :deserialize buf ptr-) (incf ptr- (send _curpos :serialization-length))
   ;; geometry_msgs/Vector3 _despos
     (send _despos :deserialize buf ptr-) (incf ptr- (send _despos :serialization-length))
   ;; geometry_msgs/Vector3 _curvel
     (send _curvel :deserialize buf ptr-) (incf ptr- (send _curvel :serialization-length))
   ;; geometry_msgs/Vector3 _desvel
     (send _desvel :deserialize buf ptr-) (incf ptr- (send _desvel :serialization-length))
   ;; geometry_msgs/Vector3 _curacc
     (send _curacc :deserialize buf ptr-) (incf ptr- (send _curacc :serialization-length))
   ;; geometry_msgs/Vector3 _desacc
     (send _desacc :deserialize buf ptr-) (incf ptr- (send _desacc :serialization-length))
   ;; geometry_msgs/Vector3 _poserr
     (send _poserr :deserialize buf ptr-) (incf ptr- (send _poserr :serialization-length))
   ;; geometry_msgs/Vector3 _velerr
     (send _velerr :deserialize buf ptr-) (incf ptr- (send _velerr :serialization-length))
   ;; geometry_msgs/Quaternion _curor
     (send _curor :deserialize buf ptr-) (incf ptr- (send _curor :serialization-length))
   ;; geometry_msgs/Quaternion _desor
     (send _desor :deserialize buf ptr-) (incf ptr- (send _desor :serialization-length))
   ;; float64 _thrust
     (setq _thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _M1_pos_err
     (setq _M1_pos_err (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _M2_pos_err
     (setq _M2_pos_err (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _M1_vel_err
     (setq _M1_vel_err (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _M2_vel_err
     (setq _M2_vel_err (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get msg_check::PlotDataMsg :md5sum-) "0d96fc0f4d709ad5a1e1b4d1fe446936")
(setf (get msg_check::PlotDataMsg :datatype-) "msg_check/PlotDataMsg")
(setf (get msg_check::PlotDataMsg :definition-)
      "Header header

geometry_msgs/Vector3 curpos
geometry_msgs/Vector3 despos

geometry_msgs/Vector3 curvel
geometry_msgs/Vector3 desvel

geometry_msgs/Vector3 curacc
geometry_msgs/Vector3 desacc

geometry_msgs/Vector3 poserr
geometry_msgs/Vector3 velerr

geometry_msgs/Quaternion curor
geometry_msgs/Quaternion desor

float64 thrust
float64 M1_pos_err
float64 M2_pos_err
float64 M1_vel_err
float64 M2_vel_err


================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :msg_check/PlotDataMsg "0d96fc0f4d709ad5a1e1b4d1fe446936")


