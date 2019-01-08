// Auto-generated. Do not edit!

// (in-package husky_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class HuskyStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.uptime = null;
      this.ros_control_loop_freq = null;
      this.mcu_and_user_port_current = null;
      this.left_driver_current = null;
      this.right_driver_current = null;
      this.battery_voltage = null;
      this.left_driver_voltage = null;
      this.right_driver_voltage = null;
      this.left_driver_temp = null;
      this.right_driver_temp = null;
      this.left_motor_temp = null;
      this.right_motor_temp = null;
      this.capacity_estimate = null;
      this.charge_estimate = null;
      this.timeout = null;
      this.lockout = null;
      this.e_stop = null;
      this.ros_pause = null;
      this.no_battery = null;
      this.current_limit = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('uptime')) {
        this.uptime = initObj.uptime
      }
      else {
        this.uptime = 0;
      }
      if (initObj.hasOwnProperty('ros_control_loop_freq')) {
        this.ros_control_loop_freq = initObj.ros_control_loop_freq
      }
      else {
        this.ros_control_loop_freq = 0.0;
      }
      if (initObj.hasOwnProperty('mcu_and_user_port_current')) {
        this.mcu_and_user_port_current = initObj.mcu_and_user_port_current
      }
      else {
        this.mcu_and_user_port_current = 0.0;
      }
      if (initObj.hasOwnProperty('left_driver_current')) {
        this.left_driver_current = initObj.left_driver_current
      }
      else {
        this.left_driver_current = 0.0;
      }
      if (initObj.hasOwnProperty('right_driver_current')) {
        this.right_driver_current = initObj.right_driver_current
      }
      else {
        this.right_driver_current = 0.0;
      }
      if (initObj.hasOwnProperty('battery_voltage')) {
        this.battery_voltage = initObj.battery_voltage
      }
      else {
        this.battery_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('left_driver_voltage')) {
        this.left_driver_voltage = initObj.left_driver_voltage
      }
      else {
        this.left_driver_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('right_driver_voltage')) {
        this.right_driver_voltage = initObj.right_driver_voltage
      }
      else {
        this.right_driver_voltage = 0.0;
      }
      if (initObj.hasOwnProperty('left_driver_temp')) {
        this.left_driver_temp = initObj.left_driver_temp
      }
      else {
        this.left_driver_temp = 0.0;
      }
      if (initObj.hasOwnProperty('right_driver_temp')) {
        this.right_driver_temp = initObj.right_driver_temp
      }
      else {
        this.right_driver_temp = 0.0;
      }
      if (initObj.hasOwnProperty('left_motor_temp')) {
        this.left_motor_temp = initObj.left_motor_temp
      }
      else {
        this.left_motor_temp = 0.0;
      }
      if (initObj.hasOwnProperty('right_motor_temp')) {
        this.right_motor_temp = initObj.right_motor_temp
      }
      else {
        this.right_motor_temp = 0.0;
      }
      if (initObj.hasOwnProperty('capacity_estimate')) {
        this.capacity_estimate = initObj.capacity_estimate
      }
      else {
        this.capacity_estimate = 0;
      }
      if (initObj.hasOwnProperty('charge_estimate')) {
        this.charge_estimate = initObj.charge_estimate
      }
      else {
        this.charge_estimate = 0.0;
      }
      if (initObj.hasOwnProperty('timeout')) {
        this.timeout = initObj.timeout
      }
      else {
        this.timeout = false;
      }
      if (initObj.hasOwnProperty('lockout')) {
        this.lockout = initObj.lockout
      }
      else {
        this.lockout = false;
      }
      if (initObj.hasOwnProperty('e_stop')) {
        this.e_stop = initObj.e_stop
      }
      else {
        this.e_stop = false;
      }
      if (initObj.hasOwnProperty('ros_pause')) {
        this.ros_pause = initObj.ros_pause
      }
      else {
        this.ros_pause = false;
      }
      if (initObj.hasOwnProperty('no_battery')) {
        this.no_battery = initObj.no_battery
      }
      else {
        this.no_battery = false;
      }
      if (initObj.hasOwnProperty('current_limit')) {
        this.current_limit = initObj.current_limit
      }
      else {
        this.current_limit = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type HuskyStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [uptime]
    bufferOffset = _serializer.uint32(obj.uptime, buffer, bufferOffset);
    // Serialize message field [ros_control_loop_freq]
    bufferOffset = _serializer.float64(obj.ros_control_loop_freq, buffer, bufferOffset);
    // Serialize message field [mcu_and_user_port_current]
    bufferOffset = _serializer.float64(obj.mcu_and_user_port_current, buffer, bufferOffset);
    // Serialize message field [left_driver_current]
    bufferOffset = _serializer.float64(obj.left_driver_current, buffer, bufferOffset);
    // Serialize message field [right_driver_current]
    bufferOffset = _serializer.float64(obj.right_driver_current, buffer, bufferOffset);
    // Serialize message field [battery_voltage]
    bufferOffset = _serializer.float64(obj.battery_voltage, buffer, bufferOffset);
    // Serialize message field [left_driver_voltage]
    bufferOffset = _serializer.float64(obj.left_driver_voltage, buffer, bufferOffset);
    // Serialize message field [right_driver_voltage]
    bufferOffset = _serializer.float64(obj.right_driver_voltage, buffer, bufferOffset);
    // Serialize message field [left_driver_temp]
    bufferOffset = _serializer.float64(obj.left_driver_temp, buffer, bufferOffset);
    // Serialize message field [right_driver_temp]
    bufferOffset = _serializer.float64(obj.right_driver_temp, buffer, bufferOffset);
    // Serialize message field [left_motor_temp]
    bufferOffset = _serializer.float64(obj.left_motor_temp, buffer, bufferOffset);
    // Serialize message field [right_motor_temp]
    bufferOffset = _serializer.float64(obj.right_motor_temp, buffer, bufferOffset);
    // Serialize message field [capacity_estimate]
    bufferOffset = _serializer.uint16(obj.capacity_estimate, buffer, bufferOffset);
    // Serialize message field [charge_estimate]
    bufferOffset = _serializer.float64(obj.charge_estimate, buffer, bufferOffset);
    // Serialize message field [timeout]
    bufferOffset = _serializer.bool(obj.timeout, buffer, bufferOffset);
    // Serialize message field [lockout]
    bufferOffset = _serializer.bool(obj.lockout, buffer, bufferOffset);
    // Serialize message field [e_stop]
    bufferOffset = _serializer.bool(obj.e_stop, buffer, bufferOffset);
    // Serialize message field [ros_pause]
    bufferOffset = _serializer.bool(obj.ros_pause, buffer, bufferOffset);
    // Serialize message field [no_battery]
    bufferOffset = _serializer.bool(obj.no_battery, buffer, bufferOffset);
    // Serialize message field [current_limit]
    bufferOffset = _serializer.bool(obj.current_limit, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type HuskyStatus
    let len;
    let data = new HuskyStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [uptime]
    data.uptime = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [ros_control_loop_freq]
    data.ros_control_loop_freq = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mcu_and_user_port_current]
    data.mcu_and_user_port_current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_driver_current]
    data.left_driver_current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_driver_current]
    data.right_driver_current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [battery_voltage]
    data.battery_voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_driver_voltage]
    data.left_driver_voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_driver_voltage]
    data.right_driver_voltage = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_driver_temp]
    data.left_driver_temp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_driver_temp]
    data.right_driver_temp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left_motor_temp]
    data.left_motor_temp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [right_motor_temp]
    data.right_motor_temp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [capacity_estimate]
    data.capacity_estimate = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [charge_estimate]
    data.charge_estimate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [timeout]
    data.timeout = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [lockout]
    data.lockout = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [e_stop]
    data.e_stop = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [ros_pause]
    data.ros_pause = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [no_battery]
    data.no_battery = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [current_limit]
    data.current_limit = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 108;
  }

  static datatype() {
    // Returns string type for a message object
    return 'husky_msgs/HuskyStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fd724379c53d89ec4629be3b235dc10d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new HuskyStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.uptime !== undefined) {
      resolved.uptime = msg.uptime;
    }
    else {
      resolved.uptime = 0
    }

    if (msg.ros_control_loop_freq !== undefined) {
      resolved.ros_control_loop_freq = msg.ros_control_loop_freq;
    }
    else {
      resolved.ros_control_loop_freq = 0.0
    }

    if (msg.mcu_and_user_port_current !== undefined) {
      resolved.mcu_and_user_port_current = msg.mcu_and_user_port_current;
    }
    else {
      resolved.mcu_and_user_port_current = 0.0
    }

    if (msg.left_driver_current !== undefined) {
      resolved.left_driver_current = msg.left_driver_current;
    }
    else {
      resolved.left_driver_current = 0.0
    }

    if (msg.right_driver_current !== undefined) {
      resolved.right_driver_current = msg.right_driver_current;
    }
    else {
      resolved.right_driver_current = 0.0
    }

    if (msg.battery_voltage !== undefined) {
      resolved.battery_voltage = msg.battery_voltage;
    }
    else {
      resolved.battery_voltage = 0.0
    }

    if (msg.left_driver_voltage !== undefined) {
      resolved.left_driver_voltage = msg.left_driver_voltage;
    }
    else {
      resolved.left_driver_voltage = 0.0
    }

    if (msg.right_driver_voltage !== undefined) {
      resolved.right_driver_voltage = msg.right_driver_voltage;
    }
    else {
      resolved.right_driver_voltage = 0.0
    }

    if (msg.left_driver_temp !== undefined) {
      resolved.left_driver_temp = msg.left_driver_temp;
    }
    else {
      resolved.left_driver_temp = 0.0
    }

    if (msg.right_driver_temp !== undefined) {
      resolved.right_driver_temp = msg.right_driver_temp;
    }
    else {
      resolved.right_driver_temp = 0.0
    }

    if (msg.left_motor_temp !== undefined) {
      resolved.left_motor_temp = msg.left_motor_temp;
    }
    else {
      resolved.left_motor_temp = 0.0
    }

    if (msg.right_motor_temp !== undefined) {
      resolved.right_motor_temp = msg.right_motor_temp;
    }
    else {
      resolved.right_motor_temp = 0.0
    }

    if (msg.capacity_estimate !== undefined) {
      resolved.capacity_estimate = msg.capacity_estimate;
    }
    else {
      resolved.capacity_estimate = 0
    }

    if (msg.charge_estimate !== undefined) {
      resolved.charge_estimate = msg.charge_estimate;
    }
    else {
      resolved.charge_estimate = 0.0
    }

    if (msg.timeout !== undefined) {
      resolved.timeout = msg.timeout;
    }
    else {
      resolved.timeout = false
    }

    if (msg.lockout !== undefined) {
      resolved.lockout = msg.lockout;
    }
    else {
      resolved.lockout = false
    }

    if (msg.e_stop !== undefined) {
      resolved.e_stop = msg.e_stop;
    }
    else {
      resolved.e_stop = false
    }

    if (msg.ros_pause !== undefined) {
      resolved.ros_pause = msg.ros_pause;
    }
    else {
      resolved.ros_pause = false
    }

    if (msg.no_battery !== undefined) {
      resolved.no_battery = msg.no_battery;
    }
    else {
      resolved.no_battery = false
    }

    if (msg.current_limit !== undefined) {
      resolved.current_limit = msg.current_limit;
    }
    else {
      resolved.current_limit = false
    }

    return resolved;
    }
};

module.exports = HuskyStatus;
