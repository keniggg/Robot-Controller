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
      this.skin1 = null;
      this.skin2 = null;
      this.total_grip_force = null;
      this.force_diff = null;
      this.left_contact = null;
      this.right_contact = null;
      this.object_grasped = null;
      this.slip_detected = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('skin1')) {
        this.skin1 = initObj.skin1
      }
      else {
        this.skin1 = new TactileFrame();
      }
      if (initObj.hasOwnProperty('skin2')) {
        this.skin2 = initObj.skin2
      }
      else {
        this.skin2 = new TactileFrame();
      }
      if (initObj.hasOwnProperty('total_grip_force')) {
        this.total_grip_force = initObj.total_grip_force
      }
      else {
        this.total_grip_force = 0.0;
      }
      if (initObj.hasOwnProperty('force_diff')) {
        this.force_diff = initObj.force_diff
      }
      else {
        this.force_diff = 0.0;
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
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TactileState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [skin1]
    bufferOffset = TactileFrame.serialize(obj.skin1, buffer, bufferOffset);
    // Serialize message field [skin2]
    bufferOffset = TactileFrame.serialize(obj.skin2, buffer, bufferOffset);
    // Serialize message field [total_grip_force]
    bufferOffset = _serializer.float32(obj.total_grip_force, buffer, bufferOffset);
    // Serialize message field [force_diff]
    bufferOffset = _serializer.float32(obj.force_diff, buffer, bufferOffset);
    // Serialize message field [left_contact]
    bufferOffset = _serializer.bool(obj.left_contact, buffer, bufferOffset);
    // Serialize message field [right_contact]
    bufferOffset = _serializer.bool(obj.right_contact, buffer, bufferOffset);
    // Serialize message field [object_grasped]
    bufferOffset = _serializer.bool(obj.object_grasped, buffer, bufferOffset);
    // Serialize message field [slip_detected]
    bufferOffset = _serializer.bool(obj.slip_detected, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TactileState
    let len;
    let data = new TactileState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [skin1]
    data.skin1 = TactileFrame.deserialize(buffer, bufferOffset);
    // Deserialize message field [skin2]
    data.skin2 = TactileFrame.deserialize(buffer, bufferOffset);
    // Deserialize message field [total_grip_force]
    data.total_grip_force = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force_diff]
    data.force_diff = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_contact]
    data.left_contact = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_contact]
    data.right_contact = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [object_grasped]
    data.object_grasped = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [slip_detected]
    data.slip_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += TactileFrame.getMessageSize(object.skin1);
    length += TactileFrame.getMessageSize(object.skin2);
    length += _getByteLength(object.status);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/TactileState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9b76704935dbe875f8fc5a941f64277a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    alicia_flexible_grasp_supervisor/TactileFrame skin1
    alicia_flexible_grasp_supervisor/TactileFrame skin2
    float32 total_grip_force
    float32 force_diff
    bool left_contact
    bool right_contact
    bool object_grasped
    bool slip_detected
    string status
    
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
    uint8 skin_id
    uint16 rows
    uint16 cols
    float32[] values
    float32 total_force
    float32 max_force
    uint16 max_index
    float32 center_x
    float32 center_y
    bool contact
    bool valid
    string status
    
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

    if (msg.skin1 !== undefined) {
      resolved.skin1 = TactileFrame.Resolve(msg.skin1)
    }
    else {
      resolved.skin1 = new TactileFrame()
    }

    if (msg.skin2 !== undefined) {
      resolved.skin2 = TactileFrame.Resolve(msg.skin2)
    }
    else {
      resolved.skin2 = new TactileFrame()
    }

    if (msg.total_grip_force !== undefined) {
      resolved.total_grip_force = msg.total_grip_force;
    }
    else {
      resolved.total_grip_force = 0.0
    }

    if (msg.force_diff !== undefined) {
      resolved.force_diff = msg.force_diff;
    }
    else {
      resolved.force_diff = 0.0
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

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    return resolved;
    }
};

module.exports = TactileState;
