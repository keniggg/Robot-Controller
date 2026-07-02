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

class TactileFrame {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.skin_name = null;
      this.values = null;
      this.rows = null;
      this.cols = null;
      this.total_force_mn = null;
      this.max_force_mn = null;
      this.max_index = null;
      this.center_x = null;
      this.center_y = null;
      this.contact = null;
      this.valid = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('skin_name')) {
        this.skin_name = initObj.skin_name
      }
      else {
        this.skin_name = '';
      }
      if (initObj.hasOwnProperty('values')) {
        this.values = initObj.values
      }
      else {
        this.values = [];
      }
      if (initObj.hasOwnProperty('rows')) {
        this.rows = initObj.rows
      }
      else {
        this.rows = 0;
      }
      if (initObj.hasOwnProperty('cols')) {
        this.cols = initObj.cols
      }
      else {
        this.cols = 0;
      }
      if (initObj.hasOwnProperty('total_force_mn')) {
        this.total_force_mn = initObj.total_force_mn
      }
      else {
        this.total_force_mn = 0.0;
      }
      if (initObj.hasOwnProperty('max_force_mn')) {
        this.max_force_mn = initObj.max_force_mn
      }
      else {
        this.max_force_mn = 0.0;
      }
      if (initObj.hasOwnProperty('max_index')) {
        this.max_index = initObj.max_index
      }
      else {
        this.max_index = 0;
      }
      if (initObj.hasOwnProperty('center_x')) {
        this.center_x = initObj.center_x
      }
      else {
        this.center_x = 0.0;
      }
      if (initObj.hasOwnProperty('center_y')) {
        this.center_y = initObj.center_y
      }
      else {
        this.center_y = 0.0;
      }
      if (initObj.hasOwnProperty('contact')) {
        this.contact = initObj.contact
      }
      else {
        this.contact = false;
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
    // Serializes a message object of type TactileFrame
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [skin_name]
    bufferOffset = _serializer.string(obj.skin_name, buffer, bufferOffset);
    // Serialize message field [values]
    bufferOffset = _arraySerializer.float32(obj.values, buffer, bufferOffset, null);
    // Serialize message field [rows]
    bufferOffset = _serializer.uint32(obj.rows, buffer, bufferOffset);
    // Serialize message field [cols]
    bufferOffset = _serializer.uint32(obj.cols, buffer, bufferOffset);
    // Serialize message field [total_force_mn]
    bufferOffset = _serializer.float32(obj.total_force_mn, buffer, bufferOffset);
    // Serialize message field [max_force_mn]
    bufferOffset = _serializer.float32(obj.max_force_mn, buffer, bufferOffset);
    // Serialize message field [max_index]
    bufferOffset = _serializer.uint32(obj.max_index, buffer, bufferOffset);
    // Serialize message field [center_x]
    bufferOffset = _serializer.float32(obj.center_x, buffer, bufferOffset);
    // Serialize message field [center_y]
    bufferOffset = _serializer.float32(obj.center_y, buffer, bufferOffset);
    // Serialize message field [contact]
    bufferOffset = _serializer.bool(obj.contact, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TactileFrame
    let len;
    let data = new TactileFrame(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [skin_name]
    data.skin_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [values]
    data.values = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [rows]
    data.rows = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [cols]
    data.cols = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [total_force_mn]
    data.total_force_mn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_force_mn]
    data.max_force_mn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_index]
    data.max_index = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [center_x]
    data.center_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [center_y]
    data.center_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [contact]
    data.contact = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.skin_name);
    length += 4 * object.values.length;
    return length + 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/TactileFrame';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a144ff6b8561cbba1f903254a0728a98';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new TactileFrame(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.skin_name !== undefined) {
      resolved.skin_name = msg.skin_name;
    }
    else {
      resolved.skin_name = ''
    }

    if (msg.values !== undefined) {
      resolved.values = msg.values;
    }
    else {
      resolved.values = []
    }

    if (msg.rows !== undefined) {
      resolved.rows = msg.rows;
    }
    else {
      resolved.rows = 0
    }

    if (msg.cols !== undefined) {
      resolved.cols = msg.cols;
    }
    else {
      resolved.cols = 0
    }

    if (msg.total_force_mn !== undefined) {
      resolved.total_force_mn = msg.total_force_mn;
    }
    else {
      resolved.total_force_mn = 0.0
    }

    if (msg.max_force_mn !== undefined) {
      resolved.max_force_mn = msg.max_force_mn;
    }
    else {
      resolved.max_force_mn = 0.0
    }

    if (msg.max_index !== undefined) {
      resolved.max_index = msg.max_index;
    }
    else {
      resolved.max_index = 0
    }

    if (msg.center_x !== undefined) {
      resolved.center_x = msg.center_x;
    }
    else {
      resolved.center_x = 0.0
    }

    if (msg.center_y !== undefined) {
      resolved.center_y = msg.center_y;
    }
    else {
      resolved.center_y = 0.0
    }

    if (msg.contact !== undefined) {
      resolved.contact = msg.contact;
    }
    else {
      resolved.contact = false
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

module.exports = TactileFrame;
