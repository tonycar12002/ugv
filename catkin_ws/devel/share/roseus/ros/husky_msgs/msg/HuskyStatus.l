;; Auto-generated. Do not edit!


(when (boundp 'husky_msgs::HuskyStatus)
  (if (not (find-package "HUSKY_MSGS"))
    (make-package "HUSKY_MSGS"))
  (shadow 'HuskyStatus (find-package "HUSKY_MSGS")))
(unless (find-package "HUSKY_MSGS::HUSKYSTATUS")
  (make-package "HUSKY_MSGS::HUSKYSTATUS"))

(in-package "ROS")
;;//! \htmlinclude HuskyStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass husky_msgs::HuskyStatus
  :super ros::object
  :slots (_header _uptime _ros_control_loop_freq _mcu_and_user_port_current _left_driver_current _right_driver_current _battery_voltage _left_driver_voltage _right_driver_voltage _left_driver_temp _right_driver_temp _left_motor_temp _right_motor_temp _capacity_estimate _charge_estimate _timeout _lockout _e_stop _ros_pause _no_battery _current_limit ))

(defmethod husky_msgs::HuskyStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:uptime __uptime) 0)
    ((:ros_control_loop_freq __ros_control_loop_freq) 0.0)
    ((:mcu_and_user_port_current __mcu_and_user_port_current) 0.0)
    ((:left_driver_current __left_driver_current) 0.0)
    ((:right_driver_current __right_driver_current) 0.0)
    ((:battery_voltage __battery_voltage) 0.0)
    ((:left_driver_voltage __left_driver_voltage) 0.0)
    ((:right_driver_voltage __right_driver_voltage) 0.0)
    ((:left_driver_temp __left_driver_temp) 0.0)
    ((:right_driver_temp __right_driver_temp) 0.0)
    ((:left_motor_temp __left_motor_temp) 0.0)
    ((:right_motor_temp __right_motor_temp) 0.0)
    ((:capacity_estimate __capacity_estimate) 0)
    ((:charge_estimate __charge_estimate) 0.0)
    ((:timeout __timeout) nil)
    ((:lockout __lockout) nil)
    ((:e_stop __e_stop) nil)
    ((:ros_pause __ros_pause) nil)
    ((:no_battery __no_battery) nil)
    ((:current_limit __current_limit) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _uptime (round __uptime))
   (setq _ros_control_loop_freq (float __ros_control_loop_freq))
   (setq _mcu_and_user_port_current (float __mcu_and_user_port_current))
   (setq _left_driver_current (float __left_driver_current))
   (setq _right_driver_current (float __right_driver_current))
   (setq _battery_voltage (float __battery_voltage))
   (setq _left_driver_voltage (float __left_driver_voltage))
   (setq _right_driver_voltage (float __right_driver_voltage))
   (setq _left_driver_temp (float __left_driver_temp))
   (setq _right_driver_temp (float __right_driver_temp))
   (setq _left_motor_temp (float __left_motor_temp))
   (setq _right_motor_temp (float __right_motor_temp))
   (setq _capacity_estimate (round __capacity_estimate))
   (setq _charge_estimate (float __charge_estimate))
   (setq _timeout __timeout)
   (setq _lockout __lockout)
   (setq _e_stop __e_stop)
   (setq _ros_pause __ros_pause)
   (setq _no_battery __no_battery)
   (setq _current_limit __current_limit)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:uptime
   (&optional __uptime)
   (if __uptime (setq _uptime __uptime)) _uptime)
  (:ros_control_loop_freq
   (&optional __ros_control_loop_freq)
   (if __ros_control_loop_freq (setq _ros_control_loop_freq __ros_control_loop_freq)) _ros_control_loop_freq)
  (:mcu_and_user_port_current
   (&optional __mcu_and_user_port_current)
   (if __mcu_and_user_port_current (setq _mcu_and_user_port_current __mcu_and_user_port_current)) _mcu_and_user_port_current)
  (:left_driver_current
   (&optional __left_driver_current)
   (if __left_driver_current (setq _left_driver_current __left_driver_current)) _left_driver_current)
  (:right_driver_current
   (&optional __right_driver_current)
   (if __right_driver_current (setq _right_driver_current __right_driver_current)) _right_driver_current)
  (:battery_voltage
   (&optional __battery_voltage)
   (if __battery_voltage (setq _battery_voltage __battery_voltage)) _battery_voltage)
  (:left_driver_voltage
   (&optional __left_driver_voltage)
   (if __left_driver_voltage (setq _left_driver_voltage __left_driver_voltage)) _left_driver_voltage)
  (:right_driver_voltage
   (&optional __right_driver_voltage)
   (if __right_driver_voltage (setq _right_driver_voltage __right_driver_voltage)) _right_driver_voltage)
  (:left_driver_temp
   (&optional __left_driver_temp)
   (if __left_driver_temp (setq _left_driver_temp __left_driver_temp)) _left_driver_temp)
  (:right_driver_temp
   (&optional __right_driver_temp)
   (if __right_driver_temp (setq _right_driver_temp __right_driver_temp)) _right_driver_temp)
  (:left_motor_temp
   (&optional __left_motor_temp)
   (if __left_motor_temp (setq _left_motor_temp __left_motor_temp)) _left_motor_temp)
  (:right_motor_temp
   (&optional __right_motor_temp)
   (if __right_motor_temp (setq _right_motor_temp __right_motor_temp)) _right_motor_temp)
  (:capacity_estimate
   (&optional __capacity_estimate)
   (if __capacity_estimate (setq _capacity_estimate __capacity_estimate)) _capacity_estimate)
  (:charge_estimate
   (&optional __charge_estimate)
   (if __charge_estimate (setq _charge_estimate __charge_estimate)) _charge_estimate)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:lockout
   (&optional __lockout)
   (if __lockout (setq _lockout __lockout)) _lockout)
  (:e_stop
   (&optional __e_stop)
   (if __e_stop (setq _e_stop __e_stop)) _e_stop)
  (:ros_pause
   (&optional __ros_pause)
   (if __ros_pause (setq _ros_pause __ros_pause)) _ros_pause)
  (:no_battery
   (&optional __no_battery)
   (if __no_battery (setq _no_battery __no_battery)) _no_battery)
  (:current_limit
   (&optional __current_limit)
   (if __current_limit (setq _current_limit __current_limit)) _current_limit)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _uptime
    4
    ;; float64 _ros_control_loop_freq
    8
    ;; float64 _mcu_and_user_port_current
    8
    ;; float64 _left_driver_current
    8
    ;; float64 _right_driver_current
    8
    ;; float64 _battery_voltage
    8
    ;; float64 _left_driver_voltage
    8
    ;; float64 _right_driver_voltage
    8
    ;; float64 _left_driver_temp
    8
    ;; float64 _right_driver_temp
    8
    ;; float64 _left_motor_temp
    8
    ;; float64 _right_motor_temp
    8
    ;; uint16 _capacity_estimate
    2
    ;; float64 _charge_estimate
    8
    ;; bool _timeout
    1
    ;; bool _lockout
    1
    ;; bool _e_stop
    1
    ;; bool _ros_pause
    1
    ;; bool _no_battery
    1
    ;; bool _current_limit
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _uptime
       (write-long _uptime s)
     ;; float64 _ros_control_loop_freq
       (sys::poke _ros_control_loop_freq (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _mcu_and_user_port_current
       (sys::poke _mcu_and_user_port_current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_driver_current
       (sys::poke _left_driver_current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_driver_current
       (sys::poke _right_driver_current (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _battery_voltage
       (sys::poke _battery_voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_driver_voltage
       (sys::poke _left_driver_voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_driver_voltage
       (sys::poke _right_driver_voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_driver_temp
       (sys::poke _left_driver_temp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_driver_temp
       (sys::poke _right_driver_temp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _left_motor_temp
       (sys::poke _left_motor_temp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _right_motor_temp
       (sys::poke _right_motor_temp (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint16 _capacity_estimate
       (write-word _capacity_estimate s)
     ;; float64 _charge_estimate
       (sys::poke _charge_estimate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _timeout
       (if _timeout (write-byte -1 s) (write-byte 0 s))
     ;; bool _lockout
       (if _lockout (write-byte -1 s) (write-byte 0 s))
     ;; bool _e_stop
       (if _e_stop (write-byte -1 s) (write-byte 0 s))
     ;; bool _ros_pause
       (if _ros_pause (write-byte -1 s) (write-byte 0 s))
     ;; bool _no_battery
       (if _no_battery (write-byte -1 s) (write-byte 0 s))
     ;; bool _current_limit
       (if _current_limit (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _uptime
     (setq _uptime (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _ros_control_loop_freq
     (setq _ros_control_loop_freq (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _mcu_and_user_port_current
     (setq _mcu_and_user_port_current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_driver_current
     (setq _left_driver_current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_driver_current
     (setq _right_driver_current (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _battery_voltage
     (setq _battery_voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_driver_voltage
     (setq _left_driver_voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_driver_voltage
     (setq _right_driver_voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_driver_temp
     (setq _left_driver_temp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_driver_temp
     (setq _right_driver_temp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _left_motor_temp
     (setq _left_motor_temp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _right_motor_temp
     (setq _right_motor_temp (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint16 _capacity_estimate
     (setq _capacity_estimate (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _charge_estimate
     (setq _charge_estimate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _timeout
     (setq _timeout (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _lockout
     (setq _lockout (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _e_stop
     (setq _e_stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _ros_pause
     (setq _ros_pause (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _no_battery
     (setq _no_battery (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _current_limit
     (setq _current_limit (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get husky_msgs::HuskyStatus :md5sum-) "fd724379c53d89ec4629be3b235dc10d")
(setf (get husky_msgs::HuskyStatus :datatype-) "husky_msgs/HuskyStatus")
(setf (get husky_msgs::HuskyStatus :definition-)
      "Header header

# MCU Uptime, in ms
uint32 uptime

# ROS Control loop frequency (PC-side)
float64 ros_control_loop_freq

# Current draw of platform components, in amps
float64 mcu_and_user_port_current
float64 left_driver_current
float64 right_driver_current

# Voltage of platform components, in volts
float64 battery_voltage
float64 left_driver_voltage
float64 right_driver_voltage

# Component temperatures, in C
float64 left_driver_temp
float64 right_driver_temp
float64 left_motor_temp
float64 right_motor_temp

# Battery capacity (Wh) and charge (%) estimate
uint16 capacity_estimate
float64 charge_estimate

# Husky error/stop conditions
bool timeout
bool lockout
bool e_stop
bool ros_pause
bool no_battery
bool current_limit

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
# 0: no frame
# 1: global frame
string frame_id

")



(provide :husky_msgs/HuskyStatus "fd724379c53d89ec4629be3b235dc10d")


