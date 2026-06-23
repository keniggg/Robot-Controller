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

class StopGraspRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.emergency = null;
    }
    else {
      if (initObj.hasOwnProperty('emergency')) {
        this.emergency = initObj.emergency
      }
      else {
        this.emergency = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StopGraspRequest
    // Serialize message field [emergency]
    bufferOffset = _serializer.bool(obj.emergency, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StopGraspRequest
    let len;
    let data = new StopGraspRequest(null);
    // Deserialize message field [emergency]
    data.emergency = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/StopGraspRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa208417948d3e0ac7bed355db544b81';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool emergency
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StopGraspRequest(null);
    if (msg.emergency !== undefined) {
      resolved.emergency = msg.emergency;
    }
    else {
      resolved.emergency = false
    }

    return resolved;
    }
};

class StopGraspResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stopped = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('stopped')) {
        this.stopped = initObj.stopped
      }
      else {
        this.stopped = false;
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
    // Serializes a message object of type StopGraspResponse
    // Serialize message field [stopped]
    bufferOffset = _serializer.bool(obj.stopped, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StopGraspResponse
    let len;
    let data = new StopGraspResponse(null);
    // Deserialize message field [stopped]
    data.stopped = _deserializer.bool(buffer, bufferOffset);
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
    return 'alicia_flexible_grasp_supervisor/StopGraspResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d863f2a8f61769977be628494e48654';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool stopped
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StopGraspResponse(null);
    if (msg.stopped !== undefined) {
      resolved.stopped = msg.stopped;
    }
    else {
      resolved.stopped = false
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
  Request: StopGraspRequest,
  Response: StopGraspResponse,
  md5sum() { return '82bc3459aaf52f4d9d44d2af0299d950'; },
  datatype() { return 'alicia_flexible_grasp_supervisor/StopGrasp'; }
};
