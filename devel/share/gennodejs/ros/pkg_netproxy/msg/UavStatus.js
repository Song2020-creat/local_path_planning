// Auto-generated. Do not edit!

// (in-package pkg_netproxy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class UavStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
      this.baro_altitude = null;
      this.tf_altitude = null;
      this.velocity_n = null;
      this.velocity_e = null;
      this.velocity_d = null;
      this.g_latitude = null;
      this.g_longitude = null;
      this.g_altitude = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('baro_altitude')) {
        this.baro_altitude = initObj.baro_altitude
      }
      else {
        this.baro_altitude = 0.0;
      }
      if (initObj.hasOwnProperty('tf_altitude')) {
        this.tf_altitude = initObj.tf_altitude
      }
      else {
        this.tf_altitude = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_n')) {
        this.velocity_n = initObj.velocity_n
      }
      else {
        this.velocity_n = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_e')) {
        this.velocity_e = initObj.velocity_e
      }
      else {
        this.velocity_e = 0.0;
      }
      if (initObj.hasOwnProperty('velocity_d')) {
        this.velocity_d = initObj.velocity_d
      }
      else {
        this.velocity_d = 0.0;
      }
      if (initObj.hasOwnProperty('g_latitude')) {
        this.g_latitude = initObj.g_latitude
      }
      else {
        this.g_latitude = 0.0;
      }
      if (initObj.hasOwnProperty('g_longitude')) {
        this.g_longitude = initObj.g_longitude
      }
      else {
        this.g_longitude = 0.0;
      }
      if (initObj.hasOwnProperty('g_altitude')) {
        this.g_altitude = initObj.g_altitude
      }
      else {
        this.g_altitude = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UavStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float64(obj.altitude, buffer, bufferOffset);
    // Serialize message field [baro_altitude]
    bufferOffset = _serializer.float32(obj.baro_altitude, buffer, bufferOffset);
    // Serialize message field [tf_altitude]
    bufferOffset = _serializer.float32(obj.tf_altitude, buffer, bufferOffset);
    // Serialize message field [velocity_n]
    bufferOffset = _serializer.float32(obj.velocity_n, buffer, bufferOffset);
    // Serialize message field [velocity_e]
    bufferOffset = _serializer.float32(obj.velocity_e, buffer, bufferOffset);
    // Serialize message field [velocity_d]
    bufferOffset = _serializer.float32(obj.velocity_d, buffer, bufferOffset);
    // Serialize message field [g_latitude]
    bufferOffset = _serializer.float64(obj.g_latitude, buffer, bufferOffset);
    // Serialize message field [g_longitude]
    bufferOffset = _serializer.float64(obj.g_longitude, buffer, bufferOffset);
    // Serialize message field [g_altitude]
    bufferOffset = _serializer.float64(obj.g_altitude, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UavStatus
    let len;
    let data = new UavStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [baro_altitude]
    data.baro_altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [tf_altitude]
    data.tf_altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_n]
    data.velocity_n = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_e]
    data.velocity_e = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [velocity_d]
    data.velocity_d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [g_latitude]
    data.g_latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g_longitude]
    data.g_longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [g_altitude]
    data.g_altitude = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 80;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pkg_netproxy/UavStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1e2778f4d8e4263c2ff410b965eb52cc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    #位姿信息
    float32 roll
    float32 pitch
    float32 yaw
    
    #GPS位置
    float64 latitude
    float64 longitude
    float64 altitude
    
    #气压计高度
    float32 baro_altitude
    #激光测高
    float32 tf_altitude
    
    #三轴速度
    float32 velocity_n
    float32 velocity_e
    float32 velocity_d
    
    #目标点信息
    float64 g_latitude
    float64 g_longitude
    float64 g_altitude
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UavStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.baro_altitude !== undefined) {
      resolved.baro_altitude = msg.baro_altitude;
    }
    else {
      resolved.baro_altitude = 0.0
    }

    if (msg.tf_altitude !== undefined) {
      resolved.tf_altitude = msg.tf_altitude;
    }
    else {
      resolved.tf_altitude = 0.0
    }

    if (msg.velocity_n !== undefined) {
      resolved.velocity_n = msg.velocity_n;
    }
    else {
      resolved.velocity_n = 0.0
    }

    if (msg.velocity_e !== undefined) {
      resolved.velocity_e = msg.velocity_e;
    }
    else {
      resolved.velocity_e = 0.0
    }

    if (msg.velocity_d !== undefined) {
      resolved.velocity_d = msg.velocity_d;
    }
    else {
      resolved.velocity_d = 0.0
    }

    if (msg.g_latitude !== undefined) {
      resolved.g_latitude = msg.g_latitude;
    }
    else {
      resolved.g_latitude = 0.0
    }

    if (msg.g_longitude !== undefined) {
      resolved.g_longitude = msg.g_longitude;
    }
    else {
      resolved.g_longitude = 0.0
    }

    if (msg.g_altitude !== undefined) {
      resolved.g_altitude = msg.g_altitude;
    }
    else {
      resolved.g_altitude = 0.0
    }

    return resolved;
    }
};

module.exports = UavStatus;
