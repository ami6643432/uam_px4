;; Auto-generated. Do not edit!


(when (boundp 'msg_check::SwDataMsg)
  (if (not (find-package "MSG_CHECK"))
    (make-package "MSG_CHECK"))
  (shadow 'SwDataMsg (find-package "MSG_CHECK")))
(unless (find-package "MSG_CHECK::SWDATAMSG")
  (make-package "MSG_CHECK::SWDATAMSG"))

(in-package "ROS")
;;//! \htmlinclude SwDataMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msg_check::SwDataMsg
  :super ros::object
  :slots (_header _sp _sq _acceleration _angular_acceleration _position_error _velocity_error _position_error_integral _angle_error _angle_rate_error _Kp0_1 _Kp1_1 _Kp0_2 _Kp1_2 _Kp0_3 _Kp1_3 _Kq0_1 _Kq1_1 _Kq2_1 _Kq0_2 _Kq1_2 _Kq2_2 _Kq0_3 _Kq1_3 _Kq2_3 _rho_p0_1 _rho_p1_1 _rho_p0_2 _rho_p1_2 _rho_p0_3 _rho_p1_3 _rho_q0_1 _rho_q1_1 _rho_q2_1 _rho_q0_2 _rho_q1_2 _rho_q2_2 _rho_q0_3 _rho_Q1_3 _rho_q2_3 _zeta_p _zeta_q _delTau_p _delTau_q _moments _thrust _hatM_1 _hatM_2 _hatM_3 ))

(defmethod msg_check::SwDataMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sp __sp) (instance geometry_msgs::Vector3 :init))
    ((:sq __sq) (instance geometry_msgs::Vector3 :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Vector3 :init))
    ((:angular_acceleration __angular_acceleration) (instance geometry_msgs::Vector3 :init))
    ((:position_error __position_error) (instance geometry_msgs::Vector3 :init))
    ((:velocity_error __velocity_error) (instance geometry_msgs::Vector3 :init))
    ((:position_error_integral __position_error_integral) (instance geometry_msgs::Vector3 :init))
    ((:angle_error __angle_error) (instance geometry_msgs::Vector3 :init))
    ((:angle_rate_error __angle_rate_error) (instance geometry_msgs::Vector3 :init))
    ((:Kp0_1 __Kp0_1) (instance geometry_msgs::Vector3 :init))
    ((:Kp1_1 __Kp1_1) (instance geometry_msgs::Vector3 :init))
    ((:Kp0_2 __Kp0_2) (instance geometry_msgs::Vector3 :init))
    ((:Kp1_2 __Kp1_2) (instance geometry_msgs::Vector3 :init))
    ((:Kp0_3 __Kp0_3) (instance geometry_msgs::Vector3 :init))
    ((:Kp1_3 __Kp1_3) (instance geometry_msgs::Vector3 :init))
    ((:Kq0_1 __Kq0_1) (instance geometry_msgs::Vector3 :init))
    ((:Kq1_1 __Kq1_1) (instance geometry_msgs::Vector3 :init))
    ((:Kq2_1 __Kq2_1) (instance geometry_msgs::Vector3 :init))
    ((:Kq0_2 __Kq0_2) (instance geometry_msgs::Vector3 :init))
    ((:Kq1_2 __Kq1_2) (instance geometry_msgs::Vector3 :init))
    ((:Kq2_2 __Kq2_2) (instance geometry_msgs::Vector3 :init))
    ((:Kq0_3 __Kq0_3) (instance geometry_msgs::Vector3 :init))
    ((:Kq1_3 __Kq1_3) (instance geometry_msgs::Vector3 :init))
    ((:Kq2_3 __Kq2_3) (instance geometry_msgs::Vector3 :init))
    ((:rho_p0_1 __rho_p0_1) (instance geometry_msgs::Vector3 :init))
    ((:rho_p1_1 __rho_p1_1) (instance geometry_msgs::Vector3 :init))
    ((:rho_p0_2 __rho_p0_2) (instance geometry_msgs::Vector3 :init))
    ((:rho_p1_2 __rho_p1_2) (instance geometry_msgs::Vector3 :init))
    ((:rho_p0_3 __rho_p0_3) (instance geometry_msgs::Vector3 :init))
    ((:rho_p1_3 __rho_p1_3) (instance geometry_msgs::Vector3 :init))
    ((:rho_q0_1 __rho_q0_1) (instance geometry_msgs::Vector3 :init))
    ((:rho_q1_1 __rho_q1_1) (instance geometry_msgs::Vector3 :init))
    ((:rho_q2_1 __rho_q2_1) (instance geometry_msgs::Vector3 :init))
    ((:rho_q0_2 __rho_q0_2) (instance geometry_msgs::Vector3 :init))
    ((:rho_q1_2 __rho_q1_2) (instance geometry_msgs::Vector3 :init))
    ((:rho_q2_2 __rho_q2_2) (instance geometry_msgs::Vector3 :init))
    ((:rho_q0_3 __rho_q0_3) (instance geometry_msgs::Vector3 :init))
    ((:rho_Q1_3 __rho_Q1_3) (instance geometry_msgs::Vector3 :init))
    ((:rho_q2_3 __rho_q2_3) (instance geometry_msgs::Vector3 :init))
    ((:zeta_p __zeta_p) (instance geometry_msgs::Vector3 :init))
    ((:zeta_q __zeta_q) (instance geometry_msgs::Vector3 :init))
    ((:delTau_p __delTau_p) (instance geometry_msgs::Vector3 :init))
    ((:delTau_q __delTau_q) (instance geometry_msgs::Vector3 :init))
    ((:moments __moments) (instance geometry_msgs::Vector3 :init))
    ((:thrust __thrust) 0.0)
    ((:hatM_1 __hatM_1) 0.0)
    ((:hatM_2 __hatM_2) 0.0)
    ((:hatM_3 __hatM_3) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _sp __sp)
   (setq _sq __sq)
   (setq _acceleration __acceleration)
   (setq _angular_acceleration __angular_acceleration)
   (setq _position_error __position_error)
   (setq _velocity_error __velocity_error)
   (setq _position_error_integral __position_error_integral)
   (setq _angle_error __angle_error)
   (setq _angle_rate_error __angle_rate_error)
   (setq _Kp0_1 __Kp0_1)
   (setq _Kp1_1 __Kp1_1)
   (setq _Kp0_2 __Kp0_2)
   (setq _Kp1_2 __Kp1_2)
   (setq _Kp0_3 __Kp0_3)
   (setq _Kp1_3 __Kp1_3)
   (setq _Kq0_1 __Kq0_1)
   (setq _Kq1_1 __Kq1_1)
   (setq _Kq2_1 __Kq2_1)
   (setq _Kq0_2 __Kq0_2)
   (setq _Kq1_2 __Kq1_2)
   (setq _Kq2_2 __Kq2_2)
   (setq _Kq0_3 __Kq0_3)
   (setq _Kq1_3 __Kq1_3)
   (setq _Kq2_3 __Kq2_3)
   (setq _rho_p0_1 __rho_p0_1)
   (setq _rho_p1_1 __rho_p1_1)
   (setq _rho_p0_2 __rho_p0_2)
   (setq _rho_p1_2 __rho_p1_2)
   (setq _rho_p0_3 __rho_p0_3)
   (setq _rho_p1_3 __rho_p1_3)
   (setq _rho_q0_1 __rho_q0_1)
   (setq _rho_q1_1 __rho_q1_1)
   (setq _rho_q2_1 __rho_q2_1)
   (setq _rho_q0_2 __rho_q0_2)
   (setq _rho_q1_2 __rho_q1_2)
   (setq _rho_q2_2 __rho_q2_2)
   (setq _rho_q0_3 __rho_q0_3)
   (setq _rho_Q1_3 __rho_Q1_3)
   (setq _rho_q2_3 __rho_q2_3)
   (setq _zeta_p __zeta_p)
   (setq _zeta_q __zeta_q)
   (setq _delTau_p __delTau_p)
   (setq _delTau_q __delTau_q)
   (setq _moments __moments)
   (setq _thrust (float __thrust))
   (setq _hatM_1 (float __hatM_1))
   (setq _hatM_2 (float __hatM_2))
   (setq _hatM_3 (float __hatM_3))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sp
   (&rest __sp)
   (if (keywordp (car __sp))
       (send* _sp __sp)
     (progn
       (if __sp (setq _sp (car __sp)))
       _sp)))
  (:sq
   (&rest __sq)
   (if (keywordp (car __sq))
       (send* _sq __sq)
     (progn
       (if __sq (setq _sq (car __sq)))
       _sq)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:angular_acceleration
   (&rest __angular_acceleration)
   (if (keywordp (car __angular_acceleration))
       (send* _angular_acceleration __angular_acceleration)
     (progn
       (if __angular_acceleration (setq _angular_acceleration (car __angular_acceleration)))
       _angular_acceleration)))
  (:position_error
   (&rest __position_error)
   (if (keywordp (car __position_error))
       (send* _position_error __position_error)
     (progn
       (if __position_error (setq _position_error (car __position_error)))
       _position_error)))
  (:velocity_error
   (&rest __velocity_error)
   (if (keywordp (car __velocity_error))
       (send* _velocity_error __velocity_error)
     (progn
       (if __velocity_error (setq _velocity_error (car __velocity_error)))
       _velocity_error)))
  (:position_error_integral
   (&rest __position_error_integral)
   (if (keywordp (car __position_error_integral))
       (send* _position_error_integral __position_error_integral)
     (progn
       (if __position_error_integral (setq _position_error_integral (car __position_error_integral)))
       _position_error_integral)))
  (:angle_error
   (&rest __angle_error)
   (if (keywordp (car __angle_error))
       (send* _angle_error __angle_error)
     (progn
       (if __angle_error (setq _angle_error (car __angle_error)))
       _angle_error)))
  (:angle_rate_error
   (&rest __angle_rate_error)
   (if (keywordp (car __angle_rate_error))
       (send* _angle_rate_error __angle_rate_error)
     (progn
       (if __angle_rate_error (setq _angle_rate_error (car __angle_rate_error)))
       _angle_rate_error)))
  (:Kp0_1
   (&rest __Kp0_1)
   (if (keywordp (car __Kp0_1))
       (send* _Kp0_1 __Kp0_1)
     (progn
       (if __Kp0_1 (setq _Kp0_1 (car __Kp0_1)))
       _Kp0_1)))
  (:Kp1_1
   (&rest __Kp1_1)
   (if (keywordp (car __Kp1_1))
       (send* _Kp1_1 __Kp1_1)
     (progn
       (if __Kp1_1 (setq _Kp1_1 (car __Kp1_1)))
       _Kp1_1)))
  (:Kp0_2
   (&rest __Kp0_2)
   (if (keywordp (car __Kp0_2))
       (send* _Kp0_2 __Kp0_2)
     (progn
       (if __Kp0_2 (setq _Kp0_2 (car __Kp0_2)))
       _Kp0_2)))
  (:Kp1_2
   (&rest __Kp1_2)
   (if (keywordp (car __Kp1_2))
       (send* _Kp1_2 __Kp1_2)
     (progn
       (if __Kp1_2 (setq _Kp1_2 (car __Kp1_2)))
       _Kp1_2)))
  (:Kp0_3
   (&rest __Kp0_3)
   (if (keywordp (car __Kp0_3))
       (send* _Kp0_3 __Kp0_3)
     (progn
       (if __Kp0_3 (setq _Kp0_3 (car __Kp0_3)))
       _Kp0_3)))
  (:Kp1_3
   (&rest __Kp1_3)
   (if (keywordp (car __Kp1_3))
       (send* _Kp1_3 __Kp1_3)
     (progn
       (if __Kp1_3 (setq _Kp1_3 (car __Kp1_3)))
       _Kp1_3)))
  (:Kq0_1
   (&rest __Kq0_1)
   (if (keywordp (car __Kq0_1))
       (send* _Kq0_1 __Kq0_1)
     (progn
       (if __Kq0_1 (setq _Kq0_1 (car __Kq0_1)))
       _Kq0_1)))
  (:Kq1_1
   (&rest __Kq1_1)
   (if (keywordp (car __Kq1_1))
       (send* _Kq1_1 __Kq1_1)
     (progn
       (if __Kq1_1 (setq _Kq1_1 (car __Kq1_1)))
       _Kq1_1)))
  (:Kq2_1
   (&rest __Kq2_1)
   (if (keywordp (car __Kq2_1))
       (send* _Kq2_1 __Kq2_1)
     (progn
       (if __Kq2_1 (setq _Kq2_1 (car __Kq2_1)))
       _Kq2_1)))
  (:Kq0_2
   (&rest __Kq0_2)
   (if (keywordp (car __Kq0_2))
       (send* _Kq0_2 __Kq0_2)
     (progn
       (if __Kq0_2 (setq _Kq0_2 (car __Kq0_2)))
       _Kq0_2)))
  (:Kq1_2
   (&rest __Kq1_2)
   (if (keywordp (car __Kq1_2))
       (send* _Kq1_2 __Kq1_2)
     (progn
       (if __Kq1_2 (setq _Kq1_2 (car __Kq1_2)))
       _Kq1_2)))
  (:Kq2_2
   (&rest __Kq2_2)
   (if (keywordp (car __Kq2_2))
       (send* _Kq2_2 __Kq2_2)
     (progn
       (if __Kq2_2 (setq _Kq2_2 (car __Kq2_2)))
       _Kq2_2)))
  (:Kq0_3
   (&rest __Kq0_3)
   (if (keywordp (car __Kq0_3))
       (send* _Kq0_3 __Kq0_3)
     (progn
       (if __Kq0_3 (setq _Kq0_3 (car __Kq0_3)))
       _Kq0_3)))
  (:Kq1_3
   (&rest __Kq1_3)
   (if (keywordp (car __Kq1_3))
       (send* _Kq1_3 __Kq1_3)
     (progn
       (if __Kq1_3 (setq _Kq1_3 (car __Kq1_3)))
       _Kq1_3)))
  (:Kq2_3
   (&rest __Kq2_3)
   (if (keywordp (car __Kq2_3))
       (send* _Kq2_3 __Kq2_3)
     (progn
       (if __Kq2_3 (setq _Kq2_3 (car __Kq2_3)))
       _Kq2_3)))
  (:rho_p0_1
   (&rest __rho_p0_1)
   (if (keywordp (car __rho_p0_1))
       (send* _rho_p0_1 __rho_p0_1)
     (progn
       (if __rho_p0_1 (setq _rho_p0_1 (car __rho_p0_1)))
       _rho_p0_1)))
  (:rho_p1_1
   (&rest __rho_p1_1)
   (if (keywordp (car __rho_p1_1))
       (send* _rho_p1_1 __rho_p1_1)
     (progn
       (if __rho_p1_1 (setq _rho_p1_1 (car __rho_p1_1)))
       _rho_p1_1)))
  (:rho_p0_2
   (&rest __rho_p0_2)
   (if (keywordp (car __rho_p0_2))
       (send* _rho_p0_2 __rho_p0_2)
     (progn
       (if __rho_p0_2 (setq _rho_p0_2 (car __rho_p0_2)))
       _rho_p0_2)))
  (:rho_p1_2
   (&rest __rho_p1_2)
   (if (keywordp (car __rho_p1_2))
       (send* _rho_p1_2 __rho_p1_2)
     (progn
       (if __rho_p1_2 (setq _rho_p1_2 (car __rho_p1_2)))
       _rho_p1_2)))
  (:rho_p0_3
   (&rest __rho_p0_3)
   (if (keywordp (car __rho_p0_3))
       (send* _rho_p0_3 __rho_p0_3)
     (progn
       (if __rho_p0_3 (setq _rho_p0_3 (car __rho_p0_3)))
       _rho_p0_3)))
  (:rho_p1_3
   (&rest __rho_p1_3)
   (if (keywordp (car __rho_p1_3))
       (send* _rho_p1_3 __rho_p1_3)
     (progn
       (if __rho_p1_3 (setq _rho_p1_3 (car __rho_p1_3)))
       _rho_p1_3)))
  (:rho_q0_1
   (&rest __rho_q0_1)
   (if (keywordp (car __rho_q0_1))
       (send* _rho_q0_1 __rho_q0_1)
     (progn
       (if __rho_q0_1 (setq _rho_q0_1 (car __rho_q0_1)))
       _rho_q0_1)))
  (:rho_q1_1
   (&rest __rho_q1_1)
   (if (keywordp (car __rho_q1_1))
       (send* _rho_q1_1 __rho_q1_1)
     (progn
       (if __rho_q1_1 (setq _rho_q1_1 (car __rho_q1_1)))
       _rho_q1_1)))
  (:rho_q2_1
   (&rest __rho_q2_1)
   (if (keywordp (car __rho_q2_1))
       (send* _rho_q2_1 __rho_q2_1)
     (progn
       (if __rho_q2_1 (setq _rho_q2_1 (car __rho_q2_1)))
       _rho_q2_1)))
  (:rho_q0_2
   (&rest __rho_q0_2)
   (if (keywordp (car __rho_q0_2))
       (send* _rho_q0_2 __rho_q0_2)
     (progn
       (if __rho_q0_2 (setq _rho_q0_2 (car __rho_q0_2)))
       _rho_q0_2)))
  (:rho_q1_2
   (&rest __rho_q1_2)
   (if (keywordp (car __rho_q1_2))
       (send* _rho_q1_2 __rho_q1_2)
     (progn
       (if __rho_q1_2 (setq _rho_q1_2 (car __rho_q1_2)))
       _rho_q1_2)))
  (:rho_q2_2
   (&rest __rho_q2_2)
   (if (keywordp (car __rho_q2_2))
       (send* _rho_q2_2 __rho_q2_2)
     (progn
       (if __rho_q2_2 (setq _rho_q2_2 (car __rho_q2_2)))
       _rho_q2_2)))
  (:rho_q0_3
   (&rest __rho_q0_3)
   (if (keywordp (car __rho_q0_3))
       (send* _rho_q0_3 __rho_q0_3)
     (progn
       (if __rho_q0_3 (setq _rho_q0_3 (car __rho_q0_3)))
       _rho_q0_3)))
  (:rho_Q1_3
   (&rest __rho_Q1_3)
   (if (keywordp (car __rho_Q1_3))
       (send* _rho_Q1_3 __rho_Q1_3)
     (progn
       (if __rho_Q1_3 (setq _rho_Q1_3 (car __rho_Q1_3)))
       _rho_Q1_3)))
  (:rho_q2_3
   (&rest __rho_q2_3)
   (if (keywordp (car __rho_q2_3))
       (send* _rho_q2_3 __rho_q2_3)
     (progn
       (if __rho_q2_3 (setq _rho_q2_3 (car __rho_q2_3)))
       _rho_q2_3)))
  (:zeta_p
   (&rest __zeta_p)
   (if (keywordp (car __zeta_p))
       (send* _zeta_p __zeta_p)
     (progn
       (if __zeta_p (setq _zeta_p (car __zeta_p)))
       _zeta_p)))
  (:zeta_q
   (&rest __zeta_q)
   (if (keywordp (car __zeta_q))
       (send* _zeta_q __zeta_q)
     (progn
       (if __zeta_q (setq _zeta_q (car __zeta_q)))
       _zeta_q)))
  (:delTau_p
   (&rest __delTau_p)
   (if (keywordp (car __delTau_p))
       (send* _delTau_p __delTau_p)
     (progn
       (if __delTau_p (setq _delTau_p (car __delTau_p)))
       _delTau_p)))
  (:delTau_q
   (&rest __delTau_q)
   (if (keywordp (car __delTau_q))
       (send* _delTau_q __delTau_q)
     (progn
       (if __delTau_q (setq _delTau_q (car __delTau_q)))
       _delTau_q)))
  (:moments
   (&rest __moments)
   (if (keywordp (car __moments))
       (send* _moments __moments)
     (progn
       (if __moments (setq _moments (car __moments)))
       _moments)))
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:hatM_1
   (&optional __hatM_1)
   (if __hatM_1 (setq _hatM_1 __hatM_1)) _hatM_1)
  (:hatM_2
   (&optional __hatM_2)
   (if __hatM_2 (setq _hatM_2 __hatM_2)) _hatM_2)
  (:hatM_3
   (&optional __hatM_3)
   (if __hatM_3 (setq _hatM_3 __hatM_3)) _hatM_3)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _sp
    (send _sp :serialization-length)
    ;; geometry_msgs/Vector3 _sq
    (send _sq :serialization-length)
    ;; geometry_msgs/Vector3 _acceleration
    (send _acceleration :serialization-length)
    ;; geometry_msgs/Vector3 _angular_acceleration
    (send _angular_acceleration :serialization-length)
    ;; geometry_msgs/Vector3 _position_error
    (send _position_error :serialization-length)
    ;; geometry_msgs/Vector3 _velocity_error
    (send _velocity_error :serialization-length)
    ;; geometry_msgs/Vector3 _position_error_integral
    (send _position_error_integral :serialization-length)
    ;; geometry_msgs/Vector3 _angle_error
    (send _angle_error :serialization-length)
    ;; geometry_msgs/Vector3 _angle_rate_error
    (send _angle_rate_error :serialization-length)
    ;; geometry_msgs/Vector3 _Kp0_1
    (send _Kp0_1 :serialization-length)
    ;; geometry_msgs/Vector3 _Kp1_1
    (send _Kp1_1 :serialization-length)
    ;; geometry_msgs/Vector3 _Kp0_2
    (send _Kp0_2 :serialization-length)
    ;; geometry_msgs/Vector3 _Kp1_2
    (send _Kp1_2 :serialization-length)
    ;; geometry_msgs/Vector3 _Kp0_3
    (send _Kp0_3 :serialization-length)
    ;; geometry_msgs/Vector3 _Kp1_3
    (send _Kp1_3 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq0_1
    (send _Kq0_1 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq1_1
    (send _Kq1_1 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq2_1
    (send _Kq2_1 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq0_2
    (send _Kq0_2 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq1_2
    (send _Kq1_2 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq2_2
    (send _Kq2_2 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq0_3
    (send _Kq0_3 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq1_3
    (send _Kq1_3 :serialization-length)
    ;; geometry_msgs/Vector3 _Kq2_3
    (send _Kq2_3 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_p0_1
    (send _rho_p0_1 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_p1_1
    (send _rho_p1_1 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_p0_2
    (send _rho_p0_2 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_p1_2
    (send _rho_p1_2 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_p0_3
    (send _rho_p0_3 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_p1_3
    (send _rho_p1_3 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_q0_1
    (send _rho_q0_1 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_q1_1
    (send _rho_q1_1 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_q2_1
    (send _rho_q2_1 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_q0_2
    (send _rho_q0_2 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_q1_2
    (send _rho_q1_2 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_q2_2
    (send _rho_q2_2 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_q0_3
    (send _rho_q0_3 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_Q1_3
    (send _rho_Q1_3 :serialization-length)
    ;; geometry_msgs/Vector3 _rho_q2_3
    (send _rho_q2_3 :serialization-length)
    ;; geometry_msgs/Vector3 _zeta_p
    (send _zeta_p :serialization-length)
    ;; geometry_msgs/Vector3 _zeta_q
    (send _zeta_q :serialization-length)
    ;; geometry_msgs/Vector3 _delTau_p
    (send _delTau_p :serialization-length)
    ;; geometry_msgs/Vector3 _delTau_q
    (send _delTau_q :serialization-length)
    ;; geometry_msgs/Vector3 _moments
    (send _moments :serialization-length)
    ;; float64 _thrust
    8
    ;; float64 _hatM_1
    8
    ;; float64 _hatM_2
    8
    ;; float64 _hatM_3
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _sp
       (send _sp :serialize s)
     ;; geometry_msgs/Vector3 _sq
       (send _sq :serialize s)
     ;; geometry_msgs/Vector3 _acceleration
       (send _acceleration :serialize s)
     ;; geometry_msgs/Vector3 _angular_acceleration
       (send _angular_acceleration :serialize s)
     ;; geometry_msgs/Vector3 _position_error
       (send _position_error :serialize s)
     ;; geometry_msgs/Vector3 _velocity_error
       (send _velocity_error :serialize s)
     ;; geometry_msgs/Vector3 _position_error_integral
       (send _position_error_integral :serialize s)
     ;; geometry_msgs/Vector3 _angle_error
       (send _angle_error :serialize s)
     ;; geometry_msgs/Vector3 _angle_rate_error
       (send _angle_rate_error :serialize s)
     ;; geometry_msgs/Vector3 _Kp0_1
       (send _Kp0_1 :serialize s)
     ;; geometry_msgs/Vector3 _Kp1_1
       (send _Kp1_1 :serialize s)
     ;; geometry_msgs/Vector3 _Kp0_2
       (send _Kp0_2 :serialize s)
     ;; geometry_msgs/Vector3 _Kp1_2
       (send _Kp1_2 :serialize s)
     ;; geometry_msgs/Vector3 _Kp0_3
       (send _Kp0_3 :serialize s)
     ;; geometry_msgs/Vector3 _Kp1_3
       (send _Kp1_3 :serialize s)
     ;; geometry_msgs/Vector3 _Kq0_1
       (send _Kq0_1 :serialize s)
     ;; geometry_msgs/Vector3 _Kq1_1
       (send _Kq1_1 :serialize s)
     ;; geometry_msgs/Vector3 _Kq2_1
       (send _Kq2_1 :serialize s)
     ;; geometry_msgs/Vector3 _Kq0_2
       (send _Kq0_2 :serialize s)
     ;; geometry_msgs/Vector3 _Kq1_2
       (send _Kq1_2 :serialize s)
     ;; geometry_msgs/Vector3 _Kq2_2
       (send _Kq2_2 :serialize s)
     ;; geometry_msgs/Vector3 _Kq0_3
       (send _Kq0_3 :serialize s)
     ;; geometry_msgs/Vector3 _Kq1_3
       (send _Kq1_3 :serialize s)
     ;; geometry_msgs/Vector3 _Kq2_3
       (send _Kq2_3 :serialize s)
     ;; geometry_msgs/Vector3 _rho_p0_1
       (send _rho_p0_1 :serialize s)
     ;; geometry_msgs/Vector3 _rho_p1_1
       (send _rho_p1_1 :serialize s)
     ;; geometry_msgs/Vector3 _rho_p0_2
       (send _rho_p0_2 :serialize s)
     ;; geometry_msgs/Vector3 _rho_p1_2
       (send _rho_p1_2 :serialize s)
     ;; geometry_msgs/Vector3 _rho_p0_3
       (send _rho_p0_3 :serialize s)
     ;; geometry_msgs/Vector3 _rho_p1_3
       (send _rho_p1_3 :serialize s)
     ;; geometry_msgs/Vector3 _rho_q0_1
       (send _rho_q0_1 :serialize s)
     ;; geometry_msgs/Vector3 _rho_q1_1
       (send _rho_q1_1 :serialize s)
     ;; geometry_msgs/Vector3 _rho_q2_1
       (send _rho_q2_1 :serialize s)
     ;; geometry_msgs/Vector3 _rho_q0_2
       (send _rho_q0_2 :serialize s)
     ;; geometry_msgs/Vector3 _rho_q1_2
       (send _rho_q1_2 :serialize s)
     ;; geometry_msgs/Vector3 _rho_q2_2
       (send _rho_q2_2 :serialize s)
     ;; geometry_msgs/Vector3 _rho_q0_3
       (send _rho_q0_3 :serialize s)
     ;; geometry_msgs/Vector3 _rho_Q1_3
       (send _rho_Q1_3 :serialize s)
     ;; geometry_msgs/Vector3 _rho_q2_3
       (send _rho_q2_3 :serialize s)
     ;; geometry_msgs/Vector3 _zeta_p
       (send _zeta_p :serialize s)
     ;; geometry_msgs/Vector3 _zeta_q
       (send _zeta_q :serialize s)
     ;; geometry_msgs/Vector3 _delTau_p
       (send _delTau_p :serialize s)
     ;; geometry_msgs/Vector3 _delTau_q
       (send _delTau_q :serialize s)
     ;; geometry_msgs/Vector3 _moments
       (send _moments :serialize s)
     ;; float64 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _hatM_1
       (sys::poke _hatM_1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _hatM_2
       (sys::poke _hatM_2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _hatM_3
       (sys::poke _hatM_3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _sp
     (send _sp :deserialize buf ptr-) (incf ptr- (send _sp :serialization-length))
   ;; geometry_msgs/Vector3 _sq
     (send _sq :deserialize buf ptr-) (incf ptr- (send _sq :serialization-length))
   ;; geometry_msgs/Vector3 _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;; geometry_msgs/Vector3 _angular_acceleration
     (send _angular_acceleration :deserialize buf ptr-) (incf ptr- (send _angular_acceleration :serialization-length))
   ;; geometry_msgs/Vector3 _position_error
     (send _position_error :deserialize buf ptr-) (incf ptr- (send _position_error :serialization-length))
   ;; geometry_msgs/Vector3 _velocity_error
     (send _velocity_error :deserialize buf ptr-) (incf ptr- (send _velocity_error :serialization-length))
   ;; geometry_msgs/Vector3 _position_error_integral
     (send _position_error_integral :deserialize buf ptr-) (incf ptr- (send _position_error_integral :serialization-length))
   ;; geometry_msgs/Vector3 _angle_error
     (send _angle_error :deserialize buf ptr-) (incf ptr- (send _angle_error :serialization-length))
   ;; geometry_msgs/Vector3 _angle_rate_error
     (send _angle_rate_error :deserialize buf ptr-) (incf ptr- (send _angle_rate_error :serialization-length))
   ;; geometry_msgs/Vector3 _Kp0_1
     (send _Kp0_1 :deserialize buf ptr-) (incf ptr- (send _Kp0_1 :serialization-length))
   ;; geometry_msgs/Vector3 _Kp1_1
     (send _Kp1_1 :deserialize buf ptr-) (incf ptr- (send _Kp1_1 :serialization-length))
   ;; geometry_msgs/Vector3 _Kp0_2
     (send _Kp0_2 :deserialize buf ptr-) (incf ptr- (send _Kp0_2 :serialization-length))
   ;; geometry_msgs/Vector3 _Kp1_2
     (send _Kp1_2 :deserialize buf ptr-) (incf ptr- (send _Kp1_2 :serialization-length))
   ;; geometry_msgs/Vector3 _Kp0_3
     (send _Kp0_3 :deserialize buf ptr-) (incf ptr- (send _Kp0_3 :serialization-length))
   ;; geometry_msgs/Vector3 _Kp1_3
     (send _Kp1_3 :deserialize buf ptr-) (incf ptr- (send _Kp1_3 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq0_1
     (send _Kq0_1 :deserialize buf ptr-) (incf ptr- (send _Kq0_1 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq1_1
     (send _Kq1_1 :deserialize buf ptr-) (incf ptr- (send _Kq1_1 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq2_1
     (send _Kq2_1 :deserialize buf ptr-) (incf ptr- (send _Kq2_1 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq0_2
     (send _Kq0_2 :deserialize buf ptr-) (incf ptr- (send _Kq0_2 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq1_2
     (send _Kq1_2 :deserialize buf ptr-) (incf ptr- (send _Kq1_2 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq2_2
     (send _Kq2_2 :deserialize buf ptr-) (incf ptr- (send _Kq2_2 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq0_3
     (send _Kq0_3 :deserialize buf ptr-) (incf ptr- (send _Kq0_3 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq1_3
     (send _Kq1_3 :deserialize buf ptr-) (incf ptr- (send _Kq1_3 :serialization-length))
   ;; geometry_msgs/Vector3 _Kq2_3
     (send _Kq2_3 :deserialize buf ptr-) (incf ptr- (send _Kq2_3 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_p0_1
     (send _rho_p0_1 :deserialize buf ptr-) (incf ptr- (send _rho_p0_1 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_p1_1
     (send _rho_p1_1 :deserialize buf ptr-) (incf ptr- (send _rho_p1_1 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_p0_2
     (send _rho_p0_2 :deserialize buf ptr-) (incf ptr- (send _rho_p0_2 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_p1_2
     (send _rho_p1_2 :deserialize buf ptr-) (incf ptr- (send _rho_p1_2 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_p0_3
     (send _rho_p0_3 :deserialize buf ptr-) (incf ptr- (send _rho_p0_3 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_p1_3
     (send _rho_p1_3 :deserialize buf ptr-) (incf ptr- (send _rho_p1_3 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_q0_1
     (send _rho_q0_1 :deserialize buf ptr-) (incf ptr- (send _rho_q0_1 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_q1_1
     (send _rho_q1_1 :deserialize buf ptr-) (incf ptr- (send _rho_q1_1 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_q2_1
     (send _rho_q2_1 :deserialize buf ptr-) (incf ptr- (send _rho_q2_1 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_q0_2
     (send _rho_q0_2 :deserialize buf ptr-) (incf ptr- (send _rho_q0_2 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_q1_2
     (send _rho_q1_2 :deserialize buf ptr-) (incf ptr- (send _rho_q1_2 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_q2_2
     (send _rho_q2_2 :deserialize buf ptr-) (incf ptr- (send _rho_q2_2 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_q0_3
     (send _rho_q0_3 :deserialize buf ptr-) (incf ptr- (send _rho_q0_3 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_Q1_3
     (send _rho_Q1_3 :deserialize buf ptr-) (incf ptr- (send _rho_Q1_3 :serialization-length))
   ;; geometry_msgs/Vector3 _rho_q2_3
     (send _rho_q2_3 :deserialize buf ptr-) (incf ptr- (send _rho_q2_3 :serialization-length))
   ;; geometry_msgs/Vector3 _zeta_p
     (send _zeta_p :deserialize buf ptr-) (incf ptr- (send _zeta_p :serialization-length))
   ;; geometry_msgs/Vector3 _zeta_q
     (send _zeta_q :deserialize buf ptr-) (incf ptr- (send _zeta_q :serialization-length))
   ;; geometry_msgs/Vector3 _delTau_p
     (send _delTau_p :deserialize buf ptr-) (incf ptr- (send _delTau_p :serialization-length))
   ;; geometry_msgs/Vector3 _delTau_q
     (send _delTau_q :deserialize buf ptr-) (incf ptr- (send _delTau_q :serialization-length))
   ;; geometry_msgs/Vector3 _moments
     (send _moments :deserialize buf ptr-) (incf ptr- (send _moments :serialization-length))
   ;; float64 _thrust
     (setq _thrust (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _hatM_1
     (setq _hatM_1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _hatM_2
     (setq _hatM_2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _hatM_3
     (setq _hatM_3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get msg_check::SwDataMsg :md5sum-) "da1fe80469bc6c23b275683797085357")
(setf (get msg_check::SwDataMsg :datatype-) "msg_check/SwDataMsg")
(setf (get msg_check::SwDataMsg :definition-)
      "Header header
geometry_msgs/Vector3 sp
geometry_msgs/Vector3 sq
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 angular_acceleration
geometry_msgs/Vector3 position_error
geometry_msgs/Vector3 velocity_error
geometry_msgs/Vector3 position_error_integral
geometry_msgs/Vector3 angle_error
geometry_msgs/Vector3 angle_rate_error
geometry_msgs/Vector3 Kp0_1
geometry_msgs/Vector3 Kp1_1
geometry_msgs/Vector3 Kp0_2
geometry_msgs/Vector3 Kp1_2
geometry_msgs/Vector3 Kp0_3
geometry_msgs/Vector3 Kp1_3
geometry_msgs/Vector3 Kq0_1
geometry_msgs/Vector3 Kq1_1
geometry_msgs/Vector3 Kq2_1
geometry_msgs/Vector3 Kq0_2
geometry_msgs/Vector3 Kq1_2
geometry_msgs/Vector3 Kq2_2
geometry_msgs/Vector3 Kq0_3
geometry_msgs/Vector3 Kq1_3
geometry_msgs/Vector3 Kq2_3
geometry_msgs/Vector3 rho_p0_1
geometry_msgs/Vector3 rho_p1_1
geometry_msgs/Vector3 rho_p0_2
geometry_msgs/Vector3 rho_p1_2
geometry_msgs/Vector3 rho_p0_3
geometry_msgs/Vector3 rho_p1_3
geometry_msgs/Vector3 rho_q0_1
geometry_msgs/Vector3 rho_q1_1
geometry_msgs/Vector3 rho_q2_1
geometry_msgs/Vector3 rho_q0_2
geometry_msgs/Vector3 rho_q1_2
geometry_msgs/Vector3 rho_q2_2
geometry_msgs/Vector3 rho_q0_3
geometry_msgs/Vector3 rho_Q1_3
geometry_msgs/Vector3 rho_q2_3
geometry_msgs/Vector3 zeta_p
geometry_msgs/Vector3 zeta_q
geometry_msgs/Vector3 delTau_p
geometry_msgs/Vector3 delTau_q
geometry_msgs/Vector3 moments
float64 thrust
float64 hatM_1
float64 hatM_2
float64 hatM_3

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
")



(provide :msg_check/SwDataMsg "da1fe80469bc6c23b275683797085357")


