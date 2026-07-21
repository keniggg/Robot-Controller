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

class StartGraspRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.execute = null;
      this.plan_id = null;
    }
    else {
      if (initObj.hasOwnProperty('execute')) {
        this.execute = initObj.execute
      }
      else {
        this.execute = false;
      }
      if (initObj.hasOwnProperty('plan_id')) {
        this.plan_id = initObj.plan_id
      }
      else {
        this.plan_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartGraspRequest
    // Serialize message field [execute]
    bufferOffset = _serializer.bool(obj.execute, buffer, bufferOffset);
    // Serialize message field [plan_id]
    bufferOffset = _serializer.string(obj.plan_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartGraspRequest
    let len;
    let data = new StartGraspRequest(null);
    // Deserialize message field [execute]
    data.execute = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [plan_id]
    data.plan_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.plan_id);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/StartGraspRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c369fa0a7af61ec00670dcab4b57bf63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool execute
    string plan_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartGraspRequest(null);
    if (msg.execute !== undefined) {
      resolved.execute = msg.execute;
    }
    else {
      resolved.execute = false
    }

    if (msg.plan_id !== undefined) {
      resolved.plan_id = msg.plan_id;
    }
    else {
      resolved.plan_id = ''
    }

    return resolved;
    }
};

class StartGraspResponse {
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
    // Serializes a message object of type StartGraspResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartGraspResponse
    let len;
    let data = new StartGraspResponse(null);
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
    return 'alicia_flexible_grasp_supervisor/StartGraspResponse';
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
    const resolved = new StartGraspResponse(null);
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
  Request: StartGraspRequest,
  Response: StartGraspResponse,
  md5sum() { return '5d246499be275f0453d1db3b1be742a6'; },
  datatype() { return 'alicia_flexible_grasp_supervisor/StartGrasp'; }
};
