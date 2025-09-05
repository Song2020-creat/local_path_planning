// Auto-generated. Do not edit!

// (in-package pkg_netproxy.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PathMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.hb30 = null;
      this.path_ok = null;
      this.min_dis = null;
      this.path_points = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('hb30')) {
        this.hb30 = initObj.hb30
      }
      else {
        this.hb30 = 0;
      }
      if (initObj.hasOwnProperty('path_ok')) {
        this.path_ok = initObj.path_ok
      }
      else {
        this.path_ok = 0;
      }
      if (initObj.hasOwnProperty('min_dis')) {
        this.min_dis = initObj.min_dis
      }
      else {
        this.min_dis = 0.0;
      }
      if (initObj.hasOwnProperty('path_points')) {
        this.path_points = initObj.path_points
      }
      else {
        this.path_points = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [hb30]
    bufferOffset = _serializer.int32(obj.hb30, buffer, bufferOffset);
    // Serialize message field [path_ok]
    bufferOffset = _serializer.int32(obj.path_ok, buffer, bufferOffset);
    // Serialize message field [min_dis]
    bufferOffset = _serializer.float32(obj.min_dis, buffer, bufferOffset);
    // Serialize message field [path_points]
    // Serialize the length for message field [path_points]
    bufferOffset = _serializer.uint32(obj.path_points.length, buffer, bufferOffset);
    obj.path_points.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point32.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathMsg
    let len;
    let data = new PathMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [hb30]
    data.hb30 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [path_ok]
    data.path_ok = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [min_dis]
    data.min_dis = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [path_points]
    // Deserialize array length for message field [path_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.path_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.path_points[i] = geometry_msgs.msg.Point32.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 12 * object.path_points.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'pkg_netproxy/PathMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8544cf96a4ba5e5ad61125bfbe08010c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 时间戳
    std_msgs/Header header
    # 是否有障碍（0=无，1=有）
    int32 hb30
    # 是否有路径（0=无，1=有）
    int32 path_ok
    # 与障碍物的最小距离
    float32 min_dis
    # 路径点数组（经纬度）
    geometry_msgs/Point32[] path_points
    
    
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
    MSG: geometry_msgs/Point32
    # This contains the position of a point in free space(with 32 bits of precision).
    # It is recommeded to use Point wherever possible instead of Point32.  
    # 
    # This recommendation is to promote interoperability.  
    #
    # This message is designed to take up less space when sending
    # lots of points at once, as in the case of a PointCloud.  
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.hb30 !== undefined) {
      resolved.hb30 = msg.hb30;
    }
    else {
      resolved.hb30 = 0
    }

    if (msg.path_ok !== undefined) {
      resolved.path_ok = msg.path_ok;
    }
    else {
      resolved.path_ok = 0
    }

    if (msg.min_dis !== undefined) {
      resolved.min_dis = msg.min_dis;
    }
    else {
      resolved.min_dis = 0.0
    }

    if (msg.path_points !== undefined) {
      resolved.path_points = new Array(msg.path_points.length);
      for (let i = 0; i < resolved.path_points.length; ++i) {
        resolved.path_points[i] = geometry_msgs.msg.Point32.Resolve(msg.path_points[i]);
      }
    }
    else {
      resolved.path_points = []
    }

    return resolved;
    }
};

module.exports = PathMsg;
