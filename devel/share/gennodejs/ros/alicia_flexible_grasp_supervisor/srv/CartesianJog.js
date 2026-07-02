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

class CartesianJogRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dx = null;
      this.dy = null;
      this.dz = null;
      this.droll = null;
      this.dpitch = null;
      this.dyaw = null;
      this.execute = null;
    }
    else {
      if (initObj.hasOwnProperty('dx')) {
        this.dx = initObj.dx
      }
      else {
        this.dx = 0.0;
      }
      if (initObj.hasOwnProperty('dy')) {
        this.dy = initObj.dy
      }
      else {
        this.dy = 0.0;
      }
      if (initObj.hasOwnProperty('dz')) {
        this.dz = initObj.dz
      }
      else {
        this.dz = 0.0;
      }
      if (initObj.hasOwnProperty('droll')) {
        this.droll = initObj.droll
      }
      else {
        this.droll = 0.0;
      }
      if (initObj.hasOwnProperty('dpitch')) {
        this.dpitch = initObj.dpitch
      }
      else {
        this.dpitch = 0.0;
      }
      if (initObj.hasOwnProperty('dyaw')) {
        this.dyaw = initObj.dyaw
      }
      else {
        this.dyaw = 0.0;
      }
      if (initObj.hasOwnProperty('execute')) {
        this.execute = initObj.execute
      }
      else {
        this.execute = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CartesianJogRequest
    // Serialize message field [dx]
    bufferOffset = _serializer.float64(obj.dx, buffer, bufferOffset);
    // Serialize message field [dy]
    bufferOffset = _serializer.float64(obj.dy, buffer, bufferOffset);
    // Serialize message field [dz]
    bufferOffset = _serializer.float64(obj.dz, buffer, bufferOffset);
    // Serialize message field [droll]
    bufferOffset = _serializer.float64(obj.droll, buffer, bufferOffset);
    // Serialize message field [dpitch]
    bufferOffset = _serializer.float64(obj.dpitch, buffer, bufferOffset);
    // Serialize message field [dyaw]
    bufferOffset = _serializer.float64(obj.dyaw, buffer, bufferOffset);
    // Serialize message field [execute]
    bufferOffset = _serializer.bool(obj.execute, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianJogRequest
    let len;
    let data = new CartesianJogRequest(null);
    // Deserialize message field [dx]
    data.dx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dy]
    data.dy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dz]
    data.dz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [droll]
    data.droll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dpitch]
    data.dpitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dyaw]
    data.dyaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [execute]
    data.execute = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/CartesianJogRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '563a5d9842521889e912b333b62d4ffd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 dx
    float64 dy
    float64 dz
    float64 droll
    float64 dpitch
    float64 dyaw
    bool execute
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CartesianJogRequest(null);
    if (msg.dx !== undefined) {
      resolved.dx = msg.dx;
    }
    else {
      resolved.dx = 0.0
    }

    if (msg.dy !== undefined) {
      resolved.dy = msg.dy;
    }
    else {
      resolved.dy = 0.0
    }

    if (msg.dz !== undefined) {
      resolved.dz = msg.dz;
    }
    else {
      resolved.dz = 0.0
    }

    if (msg.droll !== undefined) {
      resolved.droll = msg.droll;
    }
    else {
      resolved.droll = 0.0
    }

    if (msg.dpitch !== undefined) {
      resolved.dpitch = msg.dpitch;
    }
    else {
      resolved.dpitch = 0.0
    }

    if (msg.dyaw !== undefined) {
      resolved.dyaw = msg.dyaw;
    }
    else {
      resolved.dyaw = 0.0
    }

    if (msg.execute !== undefined) {
      resolved.execute = msg.execute;
    }
    else {
      resolved.execute = false
    }

    return resolved;
    }
};

class CartesianJogResponse {
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
    // Serializes a message object of type CartesianJogResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CartesianJogResponse
    let len;
    let data = new CartesianJogResponse(null);
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
    return 'alicia_flexible_grasp_supervisor/CartesianJogResponse';
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
    const resolved = new CartesianJogResponse(null);
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
  Request: CartesianJogRequest,
  Response: CartesianJogResponse,
  md5sum() { return 'fe30d6dd3aeb27b001869b9c8844bee4'; },
  datatype() { return 'alicia_flexible_grasp_supervisor/CartesianJog'; }
};
