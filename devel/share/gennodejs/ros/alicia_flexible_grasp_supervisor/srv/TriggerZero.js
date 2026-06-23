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

class TriggerZeroRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.skin_id = null;
    }
    else {
      if (initObj.hasOwnProperty('skin_id')) {
        this.skin_id = initObj.skin_id
      }
      else {
        this.skin_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TriggerZeroRequest
    // Serialize message field [skin_id]
    bufferOffset = _serializer.uint8(obj.skin_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TriggerZeroRequest
    let len;
    let data = new TriggerZeroRequest(null);
    // Deserialize message field [skin_id]
    data.skin_id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/TriggerZeroRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7618ff84be9a52700a05bf68771d2fa5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 skin_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TriggerZeroRequest(null);
    if (msg.skin_id !== undefined) {
      resolved.skin_id = msg.skin_id;
    }
    else {
      resolved.skin_id = 0
    }

    return resolved;
    }
};

class TriggerZeroResponse {
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
    // Serializes a message object of type TriggerZeroResponse
    // Serialize message field [ok]
    bufferOffset = _serializer.bool(obj.ok, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TriggerZeroResponse
    let len;
    let data = new TriggerZeroResponse(null);
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
    return 'alicia_flexible_grasp_supervisor/TriggerZeroResponse';
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
    const resolved = new TriggerZeroResponse(null);
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
  Request: TriggerZeroRequest,
  Response: TriggerZeroResponse,
  md5sum() { return 'f7d6a6f40853bc7ccc545a177690ba20'; },
  datatype() { return 'alicia_flexible_grasp_supervisor/TriggerZero'; }
};
