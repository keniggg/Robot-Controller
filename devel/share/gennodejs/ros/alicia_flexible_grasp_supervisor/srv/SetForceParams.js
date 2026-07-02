// Auto-generated. Do not edit!

// (in-package alicia_flexible_grasp_supervisor.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetForceParamsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.contact_threshold_mn = null;
      this.target_force_mn = null;
      this.max_force_mn = null;
    }
    else {
      if (initObj.hasOwnProperty('contact_threshold_mn')) {
        this.contact_threshold_mn = initObj.contact_threshold_mn
      }
      else {
        this.contact_threshold_mn = 0.0;
      }
      if (initObj.hasOwnProperty('target_force_mn')) {
        this.target_force_mn = initObj.target_force_mn
      }
      else {
        this.target_force_mn = 0.0;
      }
      if (initObj.hasOwnProperty('max_force_mn')) {
        this.max_force_mn = initObj.max_force_mn
      }
      else {
        this.max_force_mn = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetForceParamsRequest
    // Serialize message field [contact_threshold_mn]
    bufferOffset = _serializer.float32(obj.contact_threshold_mn, buffer, bufferOffset);
    // Serialize message field [target_force_mn]
    bufferOffset = _serializer.float32(obj.target_force_mn, buffer, bufferOffset);
    // Serialize message field [max_force_mn]
    bufferOffset = _serializer.float32(obj.max_force_mn, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetForceParamsRequest
    let len;
    let data = new SetForceParamsRequest(null);
    // Deserialize message field [contact_threshold_mn]
    data.contact_threshold_mn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_force_mn]
    data.target_force_mn = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_force_mn]
    data.max_force_mn = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/SetForceParamsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f34f576fe7386891eff19b8117ea851c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 contact_threshold_mn
    float32 target_force_mn
    float32 max_force_mn
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetForceParamsRequest(null);
    if (msg.contact_threshold_mn !== undefined) {
      resolved.contact_threshold_mn = msg.contact_threshold_mn;
    }
    else {
      resolved.contact_threshold_mn = 0.0
    }

    if (msg.target_force_mn !== undefined) {
      resolved.target_force_mn = msg.target_force_mn;
    }
    else {
      resolved.target_force_mn = 0.0
    }

    if (msg.max_force_mn !== undefined) {
      resolved.max_force_mn = msg.max_force_mn;
    }
    else {
      resolved.max_force_mn = 0.0
    }

    return resolved;
    }
};

class SetForceParamsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
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
    // Serializes a message object of type SetForceParamsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetForceParamsResponse
    let len;
    let data = new SetForceParamsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/SetForceParamsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetForceParamsResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
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

module.exports = {
  Request: SetForceParamsRequest,
  Response: SetForceParamsResponse,
  md5sum() { return 'b8d5117a765e68a9f9eef7b1389ac605'; },
  datatype() { return 'alicia_flexible_grasp_supervisor/SetForceParams'; }
};
