// Auto-generated. Do not edit!

// (in-package alicia_flexible_grasp_supervisor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SafetyState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.ok = null;
      this.emergency_stop = null;
      this.force_over_limit = null;
      this.robot_timeout = null;
      this.tactile_timeout = null;
      this.camera_timeout = null;
      this.planning_failed = null;
      this.level = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
      }
      if (initObj.hasOwnProperty('emergency_stop')) {
        this.emergency_stop = initObj.emergency_stop
      }
      else {
        this.emergency_stop = false;
      }
      if (initObj.hasOwnProperty('force_over_limit')) {
        this.force_over_limit = initObj.force_over_limit
      }
      else {
        this.force_over_limit = false;
      }
      if (initObj.hasOwnProperty('robot_timeout')) {
        this.robot_timeout = initObj.robot_timeout
      }
      else {
        this.robot_timeout = false;
      }
      if (initObj.hasOwnProperty('tactile_timeout')) {
        this.tactile_timeout = initObj.tactile_timeout
      }
      else {
        this.tactile_timeout = false;
      }
      if (initObj.hasOwnProperty('camera_timeout')) {
        this.camera_timeout = initObj.camera_timeout
      }
      else {
        this.camera_timeout = false;
      }
      if (initObj.hasOwnProperty('planning_failed')) {
        this.planning_failed = initObj.planning_failed
      }
      else {
        this.planning_failed = false;
      }
      if (initObj.hasOwnProperty('level')) {
        this.level = initObj.level
      }
      else {
        this.level = '';
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SafetyState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    // Serialize message field [emergency_stop]
    bufferOffset = _serializer.bool(obj.emergency_stop, buffer, bufferOffset);
    // Serialize message field [force_over_limit]
    bufferOffset = _serializer.bool(obj.force_over_limit, buffer, bufferOffset);
    // Serialize message field [robot_timeout]
    bufferOffset = _serializer.bool(obj.robot_timeout, buffer, bufferOffset);
    // Serialize message field [tactile_timeout]
    bufferOffset = _serializer.bool(obj.tactile_timeout, buffer, bufferOffset);
    // Serialize message field [camera_timeout]
    bufferOffset = _serializer.bool(obj.camera_timeout, buffer, bufferOffset);
    // Serialize message field [planning_failed]
    bufferOffset = _serializer.bool(obj.planning_failed, buffer, bufferOffset);
    // Serialize message field [level]
    bufferOffset = _serializer.string(obj.level, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SafetyState
    let len;
    let data = new SafetyState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [emergency_stop]
    data.emergency_stop = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [force_over_limit]
    data.force_over_limit = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [robot_timeout]
    data.robot_timeout = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tactile_timeout]
    data.tactile_timeout = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [camera_timeout]
    data.camera_timeout = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [planning_failed]
    data.planning_failed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [level]
    data.level = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.level);
    length += _getByteLength(object.message);
    return length + 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/SafetyState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd422c2c3e9933ef7c15274804ca0c2f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool ok
    bool emergency_stop
    bool force_over_limit
    bool robot_timeout
    bool tactile_timeout
    bool camera_timeout
    bool planning_failed
    string level
    string message
    
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
    const resolved = new SafetyState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = false
    }

    if (msg.emergency_stop !== undefined) {
      resolved.emergency_stop = msg.emergency_stop;
    }
    else {
      resolved.emergency_stop = false
    }

    if (msg.force_over_limit !== undefined) {
      resolved.force_over_limit = msg.force_over_limit;
    }
    else {
      resolved.force_over_limit = false
    }

    if (msg.robot_timeout !== undefined) {
      resolved.robot_timeout = msg.robot_timeout;
    }
    else {
      resolved.robot_timeout = false
    }

    if (msg.tactile_timeout !== undefined) {
      resolved.tactile_timeout = msg.tactile_timeout;
    }
    else {
      resolved.tactile_timeout = false
    }

    if (msg.camera_timeout !== undefined) {
      resolved.camera_timeout = msg.camera_timeout;
    }
    else {
      resolved.camera_timeout = false
    }

    if (msg.planning_failed !== undefined) {
      resolved.planning_failed = msg.planning_failed;
    }
    else {
      resolved.planning_failed = false
    }

    if (msg.level !== undefined) {
      resolved.level = msg.level;
    }
    else {
      resolved.level = ''
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = SafetyState;
