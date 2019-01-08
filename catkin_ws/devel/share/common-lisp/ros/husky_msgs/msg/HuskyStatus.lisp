; Auto-generated. Do not edit!


(cl:in-package husky_msgs-msg)


;//! \htmlinclude HuskyStatus.msg.html

(cl:defclass <HuskyStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (uptime
    :reader uptime
    :initarg :uptime
    :type cl:integer
    :initform 0)
   (ros_control_loop_freq
    :reader ros_control_loop_freq
    :initarg :ros_control_loop_freq
    :type cl:float
    :initform 0.0)
   (mcu_and_user_port_current
    :reader mcu_and_user_port_current
    :initarg :mcu_and_user_port_current
    :type cl:float
    :initform 0.0)
   (left_driver_current
    :reader left_driver_current
    :initarg :left_driver_current
    :type cl:float
    :initform 0.0)
   (right_driver_current
    :reader right_driver_current
    :initarg :right_driver_current
    :type cl:float
    :initform 0.0)
   (battery_voltage
    :reader battery_voltage
    :initarg :battery_voltage
    :type cl:float
    :initform 0.0)
   (left_driver_voltage
    :reader left_driver_voltage
    :initarg :left_driver_voltage
    :type cl:float
    :initform 0.0)
   (right_driver_voltage
    :reader right_driver_voltage
    :initarg :right_driver_voltage
    :type cl:float
    :initform 0.0)
   (left_driver_temp
    :reader left_driver_temp
    :initarg :left_driver_temp
    :type cl:float
    :initform 0.0)
   (right_driver_temp
    :reader right_driver_temp
    :initarg :right_driver_temp
    :type cl:float
    :initform 0.0)
   (left_motor_temp
    :reader left_motor_temp
    :initarg :left_motor_temp
    :type cl:float
    :initform 0.0)
   (right_motor_temp
    :reader right_motor_temp
    :initarg :right_motor_temp
    :type cl:float
    :initform 0.0)
   (capacity_estimate
    :reader capacity_estimate
    :initarg :capacity_estimate
    :type cl:fixnum
    :initform 0)
   (charge_estimate
    :reader charge_estimate
    :initarg :charge_estimate
    :type cl:float
    :initform 0.0)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:boolean
    :initform cl:nil)
   (lockout
    :reader lockout
    :initarg :lockout
    :type cl:boolean
    :initform cl:nil)
   (e_stop
    :reader e_stop
    :initarg :e_stop
    :type cl:boolean
    :initform cl:nil)
   (ros_pause
    :reader ros_pause
    :initarg :ros_pause
    :type cl:boolean
    :initform cl:nil)
   (no_battery
    :reader no_battery
    :initarg :no_battery
    :type cl:boolean
    :initform cl:nil)
   (current_limit
    :reader current_limit
    :initarg :current_limit
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass HuskyStatus (<HuskyStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <HuskyStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'HuskyStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name husky_msgs-msg:<HuskyStatus> is deprecated: use husky_msgs-msg:HuskyStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:header-val is deprecated.  Use husky_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'uptime-val :lambda-list '(m))
(cl:defmethod uptime-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:uptime-val is deprecated.  Use husky_msgs-msg:uptime instead.")
  (uptime m))

(cl:ensure-generic-function 'ros_control_loop_freq-val :lambda-list '(m))
(cl:defmethod ros_control_loop_freq-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:ros_control_loop_freq-val is deprecated.  Use husky_msgs-msg:ros_control_loop_freq instead.")
  (ros_control_loop_freq m))

(cl:ensure-generic-function 'mcu_and_user_port_current-val :lambda-list '(m))
(cl:defmethod mcu_and_user_port_current-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:mcu_and_user_port_current-val is deprecated.  Use husky_msgs-msg:mcu_and_user_port_current instead.")
  (mcu_and_user_port_current m))

(cl:ensure-generic-function 'left_driver_current-val :lambda-list '(m))
(cl:defmethod left_driver_current-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:left_driver_current-val is deprecated.  Use husky_msgs-msg:left_driver_current instead.")
  (left_driver_current m))

(cl:ensure-generic-function 'right_driver_current-val :lambda-list '(m))
(cl:defmethod right_driver_current-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:right_driver_current-val is deprecated.  Use husky_msgs-msg:right_driver_current instead.")
  (right_driver_current m))

(cl:ensure-generic-function 'battery_voltage-val :lambda-list '(m))
(cl:defmethod battery_voltage-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:battery_voltage-val is deprecated.  Use husky_msgs-msg:battery_voltage instead.")
  (battery_voltage m))

(cl:ensure-generic-function 'left_driver_voltage-val :lambda-list '(m))
(cl:defmethod left_driver_voltage-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:left_driver_voltage-val is deprecated.  Use husky_msgs-msg:left_driver_voltage instead.")
  (left_driver_voltage m))

(cl:ensure-generic-function 'right_driver_voltage-val :lambda-list '(m))
(cl:defmethod right_driver_voltage-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:right_driver_voltage-val is deprecated.  Use husky_msgs-msg:right_driver_voltage instead.")
  (right_driver_voltage m))

(cl:ensure-generic-function 'left_driver_temp-val :lambda-list '(m))
(cl:defmethod left_driver_temp-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:left_driver_temp-val is deprecated.  Use husky_msgs-msg:left_driver_temp instead.")
  (left_driver_temp m))

(cl:ensure-generic-function 'right_driver_temp-val :lambda-list '(m))
(cl:defmethod right_driver_temp-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:right_driver_temp-val is deprecated.  Use husky_msgs-msg:right_driver_temp instead.")
  (right_driver_temp m))

(cl:ensure-generic-function 'left_motor_temp-val :lambda-list '(m))
(cl:defmethod left_motor_temp-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:left_motor_temp-val is deprecated.  Use husky_msgs-msg:left_motor_temp instead.")
  (left_motor_temp m))

(cl:ensure-generic-function 'right_motor_temp-val :lambda-list '(m))
(cl:defmethod right_motor_temp-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:right_motor_temp-val is deprecated.  Use husky_msgs-msg:right_motor_temp instead.")
  (right_motor_temp m))

(cl:ensure-generic-function 'capacity_estimate-val :lambda-list '(m))
(cl:defmethod capacity_estimate-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:capacity_estimate-val is deprecated.  Use husky_msgs-msg:capacity_estimate instead.")
  (capacity_estimate m))

(cl:ensure-generic-function 'charge_estimate-val :lambda-list '(m))
(cl:defmethod charge_estimate-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:charge_estimate-val is deprecated.  Use husky_msgs-msg:charge_estimate instead.")
  (charge_estimate m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:timeout-val is deprecated.  Use husky_msgs-msg:timeout instead.")
  (timeout m))

(cl:ensure-generic-function 'lockout-val :lambda-list '(m))
(cl:defmethod lockout-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:lockout-val is deprecated.  Use husky_msgs-msg:lockout instead.")
  (lockout m))

(cl:ensure-generic-function 'e_stop-val :lambda-list '(m))
(cl:defmethod e_stop-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:e_stop-val is deprecated.  Use husky_msgs-msg:e_stop instead.")
  (e_stop m))

(cl:ensure-generic-function 'ros_pause-val :lambda-list '(m))
(cl:defmethod ros_pause-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:ros_pause-val is deprecated.  Use husky_msgs-msg:ros_pause instead.")
  (ros_pause m))

(cl:ensure-generic-function 'no_battery-val :lambda-list '(m))
(cl:defmethod no_battery-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:no_battery-val is deprecated.  Use husky_msgs-msg:no_battery instead.")
  (no_battery m))

(cl:ensure-generic-function 'current_limit-val :lambda-list '(m))
(cl:defmethod current_limit-val ((m <HuskyStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader husky_msgs-msg:current_limit-val is deprecated.  Use husky_msgs-msg:current_limit instead.")
  (current_limit m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <HuskyStatus>) ostream)
  "Serializes a message object of type '<HuskyStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uptime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uptime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uptime)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uptime)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ros_control_loop_freq))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mcu_and_user_port_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_driver_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_driver_current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'battery_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_driver_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_driver_voltage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_driver_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_driver_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_motor_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_motor_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'capacity_estimate)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'capacity_estimate)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'charge_estimate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'timeout) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'lockout) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'e_stop) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ros_pause) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'no_battery) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'current_limit) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <HuskyStatus>) istream)
  "Deserializes a message object of type '<HuskyStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'uptime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'uptime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'uptime)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'uptime)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ros_control_loop_freq) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mcu_and_user_port_current) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_driver_current) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_driver_current) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery_voltage) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_driver_voltage) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_driver_voltage) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_driver_temp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_driver_temp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_motor_temp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_motor_temp) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'capacity_estimate)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'capacity_estimate)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'charge_estimate) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'timeout) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'lockout) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'e_stop) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'ros_pause) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'no_battery) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'current_limit) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<HuskyStatus>)))
  "Returns string type for a message object of type '<HuskyStatus>"
  "husky_msgs/HuskyStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'HuskyStatus)))
  "Returns string type for a message object of type 'HuskyStatus"
  "husky_msgs/HuskyStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<HuskyStatus>)))
  "Returns md5sum for a message object of type '<HuskyStatus>"
  "fd724379c53d89ec4629be3b235dc10d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'HuskyStatus)))
  "Returns md5sum for a message object of type 'HuskyStatus"
  "fd724379c53d89ec4629be3b235dc10d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<HuskyStatus>)))
  "Returns full string definition for message of type '<HuskyStatus>"
  (cl:format cl:nil "Header header~%~%# MCU Uptime, in ms~%uint32 uptime~%~%# ROS Control loop frequency (PC-side)~%float64 ros_control_loop_freq~%~%# Current draw of platform components, in amps~%float64 mcu_and_user_port_current~%float64 left_driver_current~%float64 right_driver_current~%~%# Voltage of platform components, in volts~%float64 battery_voltage~%float64 left_driver_voltage~%float64 right_driver_voltage~%~%# Component temperatures, in C~%float64 left_driver_temp~%float64 right_driver_temp~%float64 left_motor_temp~%float64 right_motor_temp~%~%# Battery capacity (Wh) and charge (%) estimate~%uint16 capacity_estimate~%float64 charge_estimate~%~%# Husky error/stop conditions~%bool timeout~%bool lockout~%bool e_stop~%bool ros_pause~%bool no_battery~%bool current_limit~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'HuskyStatus)))
  "Returns full string definition for message of type 'HuskyStatus"
  (cl:format cl:nil "Header header~%~%# MCU Uptime, in ms~%uint32 uptime~%~%# ROS Control loop frequency (PC-side)~%float64 ros_control_loop_freq~%~%# Current draw of platform components, in amps~%float64 mcu_and_user_port_current~%float64 left_driver_current~%float64 right_driver_current~%~%# Voltage of platform components, in volts~%float64 battery_voltage~%float64 left_driver_voltage~%float64 right_driver_voltage~%~%# Component temperatures, in C~%float64 left_driver_temp~%float64 right_driver_temp~%float64 left_motor_temp~%float64 right_motor_temp~%~%# Battery capacity (Wh) and charge (%) estimate~%uint16 capacity_estimate~%float64 charge_estimate~%~%# Husky error/stop conditions~%bool timeout~%bool lockout~%bool e_stop~%bool ros_pause~%bool no_battery~%bool current_limit~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <HuskyStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     8
     2
     8
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <HuskyStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'HuskyStatus
    (cl:cons ':header (header msg))
    (cl:cons ':uptime (uptime msg))
    (cl:cons ':ros_control_loop_freq (ros_control_loop_freq msg))
    (cl:cons ':mcu_and_user_port_current (mcu_and_user_port_current msg))
    (cl:cons ':left_driver_current (left_driver_current msg))
    (cl:cons ':right_driver_current (right_driver_current msg))
    (cl:cons ':battery_voltage (battery_voltage msg))
    (cl:cons ':left_driver_voltage (left_driver_voltage msg))
    (cl:cons ':right_driver_voltage (right_driver_voltage msg))
    (cl:cons ':left_driver_temp (left_driver_temp msg))
    (cl:cons ':right_driver_temp (right_driver_temp msg))
    (cl:cons ':left_motor_temp (left_motor_temp msg))
    (cl:cons ':right_motor_temp (right_motor_temp msg))
    (cl:cons ':capacity_estimate (capacity_estimate msg))
    (cl:cons ':charge_estimate (charge_estimate msg))
    (cl:cons ':timeout (timeout msg))
    (cl:cons ':lockout (lockout msg))
    (cl:cons ':e_stop (e_stop msg))
    (cl:cons ':ros_pause (ros_pause msg))
    (cl:cons ':no_battery (no_battery msg))
    (cl:cons ':current_limit (current_limit msg))
))
