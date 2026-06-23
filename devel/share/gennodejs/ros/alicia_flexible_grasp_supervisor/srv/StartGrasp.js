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
      this.use_latest_object = null;
      this.object_label = null;
    }
    else {
      if (initObj.hasOwnProperty('use_latest_object')) {
        this.use_latest_object = initObj.use_latest_object
      }
      else {
        this.use_latest_object = false;
      }
      if (initObj.hasOwnProperty('object_label')) {
        this.object_label = initObj.object_label
      }
      else {
        this.object_label = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type StartGraspRequest
    // Serialize message field [use_latest_object]
    bufferOffset = _serializer.bool(obj.use_latest_object, buffer, bufferOffset);
    // Serialize message field [object_label]
    bufferOffset = _serializer.string(obj.object_label, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartGraspRequest
    let len;
    let data = new StartGraspRequest(null);
    // Deserialize message field [use_latest_object]
    data.use_latest_object = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [object_label]
    data.object_label = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.object_label);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/StartGraspRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8ddc0b68e4582dfda5c94d85249f9a8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool use_latest_object
    string object_label
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartGraspRequest(null);
    if (msg.use_latest_object !== undefined) {
      resolved.use_latest_object = msg.use_latest_object;
    }
    else {
      resolved.use_latest_object = false
    }

    if (msg.object_label !== undefined) {
      resolved.object_label = msg.object_label;
    }
    else {
      resolved.object_label = ''
    }

    return resolved;
    }
};

class StartGraspResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accepted = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('accepted')) {
        this.accepted = initObj.accepted
      }
      else {
        this.accepted = false;
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
    // Serialize message field [accepted]
    bufferOffset = _serializer.bool(obj.accepted, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type StartGraspResponse
    let len;
    let data = new StartGraspResponse(null);
    // Deserialize message field [accepted]
    data.accepted = _deserializer.bool(buffer, bufferOffset);
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
    return '5273b27f74d9c0fd6d8a58d787fc7be7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool accepted
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new StartGraspResponse(null);
    if (msg.accepted !== undefined) {
      resolved.accepted = msg.accepted;
    }
    else {
      resolved.accepted = false
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
  md5sum() { return '444b03b3725d049d9ce9ac0a3dfa2c54'; },
  datatype() { return 'alicia_flexible_grasp_supervisor/StartGrasp'; }
};
