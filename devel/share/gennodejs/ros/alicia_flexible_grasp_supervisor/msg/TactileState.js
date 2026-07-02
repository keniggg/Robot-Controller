// Auto-generated. Do not edit!

// (in-package alicia_flexible_grasp_supervisor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TactileFrame = require('./TactileFrame.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class TactileState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.left = null;
      this.right = null;
      this.total_grip_force_mn = null;
      this.force_diff_mn = null;
      this.left_contact = null;
      this.right_contact = null;
      this.object_grasped = null;
      this.slip_detected = null;
      this.valid = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = new TactileFrame();
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = new TactileFrame();
      }
      if (initObj.hasOwnProperty('total_grip_force_mn')) {
        this.total_grip_force_mn = initObj.total_grip_force_mn
      }
      else {
        this.total_grip_force_mn = 0.0;
      }
      if (initObj.hasOwnProperty('force_diff_mn')) {
        this.force_diff_mn = initObj.force_diff_mn
      }
      else {
        this.force_diff_mn = 0.0;
      }
      if (initObj.hasOwnProperty('left_contact')) {
        this.left_contact = initObj.left_contact
      }
      else {
        this.left_contact = false;
      }
      if (initObj.hasOwnProperty('right_contact')) {
        this.right_contact = initObj.right_contact
      }
      else {
        this.right_contact = false;
      }
      if (initObj.hasOwnProperty('object_grasped')) {
        this.object_grasped = initObj.object_grasped
      }
      else {
        this.object_grasped = false;
      }
      if (initObj.hasOwnProperty('slip_detected')) {
        this.slip_detected = initObj.slip_detected
      }
      else {
        this.slip_detected = false;
      }
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TactileState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [left]
    bufferOffset = TactileFrame.serialize(obj.left, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = TactileFrame.serialize(obj.right, buffer, bufferOffset);
    // Serialize message field [total_grip_force_mn]
    bufferOffset = _serializer.float32(obj.total_grip_force_mn, buffer, bufferOffset);
    // Serialize message field [force_diff_mn]
    bufferOffset = _serializer.float32(obj.force_diff_mn, buffer, bufferOffset);
    // Serialize message field [left_contact]
    bufferOffset = _serializer.bool(obj.left_contact, buffer, bufferOffset);
    // Serialize message field [right_contact]
    bufferOffset = _serializer.bool(obj.right_contact, buffer, bufferOffset);
    // Serialize message field [object_grasped]
    bufferOffset = _serializer.bool(obj.object_grasped, buffer, bufferOffset);
    // Serialize message field [slip_detected]
    bufferOffset = _serializer.bool(obj.slip_detected, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TactileState
    let len;
    let data = new TactileState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [left]
    data.left = TactileFrame.deserialize(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = TactileFrame.deserialize(buffer, bufferOffset);
    // Deserialize message field [total_grip_force_mn]
    data.total_grip_force_mn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force_diff_mn]
    data.force_diff_mn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_contact]
    data.left_contact = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_contact]
    data.right_contact = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [object_grasped]
    data.object_grasped = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [slip_detected]
    data.slip_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += TactileFrame.getMessageSize(object.left);
    length += TactileFrame.getMessageSize(object.right);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/TactileState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5df6a8c43bd865ec0ec8d2f74fe1aa66';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    alicia_flexible_grasp_supervisor/TactileFrame left
    alicia_flexible_grasp_supervisor/TactileFrame right
    float32 total_grip_force_mn
    float32 force_diff_mn
    bool left_contact
    bool right_contact
    bool object_grasped
    bool slip_detected
    bool valid
    
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
    MSG: alicia_flexible_grasp_supervisor/TactileFrame
    std_msgs/Header header
    string skin_name
    float32[] values
    uint32 rows
    uint32 cols
    float32 total_force_mn
    float32 max_force_mn
    uint32 max_index
    float32 center_x
    float32 center_y
    bool contact
    bool valid
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TactileState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.left !== undefined) {
      resolved.left = TactileFrame.Resolve(msg.left)
    }
    else {
      resolved.left = new TactileFrame()
    }

    if (msg.right !== undefined) {
      resolved.right = TactileFrame.Resolve(msg.right)
    }
    else {
      resolved.right = new TactileFrame()
    }

    if (msg.total_grip_force_mn !== undefined) {
      resolved.total_grip_force_mn = msg.total_grip_force_mn;
    }
    else {
      resolved.total_grip_force_mn = 0.0
    }

    if (msg.force_diff_mn !== undefined) {
      resolved.force_diff_mn = msg.force_diff_mn;
    }
    else {
      resolved.force_diff_mn = 0.0
    }

    if (msg.left_contact !== undefined) {
      resolved.left_contact = msg.left_contact;
    }
    else {
      resolved.left_contact = false
    }

    if (msg.right_contact !== undefined) {
      resolved.right_contact = msg.right_contact;
    }
    else {
      resolved.right_contact = false
    }

    if (msg.object_grasped !== undefined) {
      resolved.object_grasped = msg.object_grasped;
    }
    else {
      resolved.object_grasped = false
    }

    if (msg.slip_detected !== undefined) {
      resolved.slip_detected = msg.slip_detected;
    }
    else {
      resolved.slip_detected = false
    }

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = false
    }

    return resolved;
    }
};

module.exports = TactileState;
