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
      this.contact_threshold = null;
      this.target_force = null;
      this.max_force = null;
    }
    else {
      if (initObj.hasOwnProperty('contact_threshold')) {
        this.contact_threshold = initObj.contact_threshold
      }
      else {
        this.contact_threshold = 0.0;
      }
      if (initObj.hasOwnProperty('target_force')) {
        this.target_force = initObj.target_force
      }
      else {
        this.target_force = 0.0;
      }
      if (initObj.hasOwnProperty('max_force')) {
        this.max_force = initObj.max_force
      }
      else {
        this.max_force = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetForceParamsRequest
    // Serialize message field [contact_threshold]
    bufferOffset = _serializer.float32(obj.contact_threshold, buffer, bufferOffset);
    // Serialize message field [target_force]
    bufferOffset = _serializer.float32(obj.target_force, buffer, bufferOffset);
    // Serialize message field [max_force]
    bufferOffset = _serializer.float32(obj.max_force, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetForceParamsRequest
    let len;
    let data = new SetForceParamsRequest(null);
    // Deserialize message field [contact_threshold]
    data.contact_threshold = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_force]
    data.target_force = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [max_force]
    data.max_force = _deserializer.float32(buffer, bufferOffset);
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
    return '53d772afe2b788b2798fa44d853c5379';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 contact_threshold
    float32 target_force
    float32 max_force
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetForceParamsRequest(null);
    if (msg.contact_threshold !== undefined) {
      resolved.contact_threshold = msg.contact_threshold;
    }
    else {
      resolved.contact_threshold = 0.0
    }

    if (msg.target_force !== undefined) {
      resolved.target_force = msg.target_force;
    }
    else {
      resolved.target_force = 0.0
    }

    if (msg.max_force !== undefined) {
      resolved.max_force = msg.max_force;
    }
    else {
      resolved.max_force = 0.0
    }

    return resolved;
    }
};

class SetForceParamsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ok = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('ok')) {
        this.ok = initObj.ok
      }
      else {
        this.ok = false;
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
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetForceParamsResponse
    let len;
    let data = new SetForceParamsResponse(null);
    // Deserialize message field [ok]
    data.ok = _deserializer.bool(buffer, bufferOffset);
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
    return 'f6fcb3b1ed8c7743c7fb7d5bcca28513';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ok
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetForceParamsResponse(null);
    if (msg.ok !== undefined) {
      resolved.ok = msg.ok;
    }
    else {
      resolved.ok = false
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
  md5sum() { return '5ef41667b487a681d2b7b91f8a1e0b10'; },
  datatype() { return 'alicia_flexible_grasp_supervisor/SetForceParams'; }
};
