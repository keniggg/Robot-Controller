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

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class TcpCalibrationCommandRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TcpCalibrationCommandRequest
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TcpCalibrationCommandRequest
    let len;
    let data = new TcpCalibrationCommandRequest(null);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.command);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/TcpCalibrationCommandRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cba5e21e920a3a2b7b375cb65b64cdea';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string command
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TcpCalibrationCommandRequest(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    return resolved;
    }
};

class TcpCalibrationCommandResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.sample_count = null;
      this.tcp_translation = null;
      this.fixed_point = null;
      this.rms_error_m = null;
      this.max_error_m = null;
      this.orientation_span_deg = null;
      this.result_file = null;
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
      if (initObj.hasOwnProperty('sample_count')) {
        this.sample_count = initObj.sample_count
      }
      else {
        this.sample_count = 0;
      }
      if (initObj.hasOwnProperty('tcp_translation')) {
        this.tcp_translation = initObj.tcp_translation
      }
      else {
        this.tcp_translation = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('fixed_point')) {
        this.fixed_point = initObj.fixed_point
      }
      else {
        this.fixed_point = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rms_error_m')) {
        this.rms_error_m = initObj.rms_error_m
      }
      else {
        this.rms_error_m = 0.0;
      }
      if (initObj.hasOwnProperty('max_error_m')) {
        this.max_error_m = initObj.max_error_m
      }
      else {
        this.max_error_m = 0.0;
      }
      if (initObj.hasOwnProperty('orientation_span_deg')) {
        this.orientation_span_deg = initObj.orientation_span_deg
      }
      else {
        this.orientation_span_deg = 0.0;
      }
      if (initObj.hasOwnProperty('result_file')) {
        this.result_file = initObj.result_file
      }
      else {
        this.result_file = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TcpCalibrationCommandResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [sample_count]
    bufferOffset = _serializer.uint32(obj.sample_count, buffer, bufferOffset);
    // Serialize message field [tcp_translation]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.tcp_translation, buffer, bufferOffset);
    // Serialize message field [fixed_point]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.fixed_point, buffer, bufferOffset);
    // Serialize message field [rms_error_m]
    bufferOffset = _serializer.float64(obj.rms_error_m, buffer, bufferOffset);
    // Serialize message field [max_error_m]
    bufferOffset = _serializer.float64(obj.max_error_m, buffer, bufferOffset);
    // Serialize message field [orientation_span_deg]
    bufferOffset = _serializer.float64(obj.orientation_span_deg, buffer, bufferOffset);
    // Serialize message field [result_file]
    bufferOffset = _serializer.string(obj.result_file, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TcpCalibrationCommandResponse
    let len;
    let data = new TcpCalibrationCommandResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [sample_count]
    data.sample_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [tcp_translation]
    data.tcp_translation = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [fixed_point]
    data.fixed_point = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rms_error_m]
    data.rms_error_m = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [max_error_m]
    data.max_error_m = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [orientation_span_deg]
    data.orientation_span_deg = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [result_file]
    data.result_file = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    length += _getByteLength(object.result_file);
    return length + 85;
  }

  static datatype() {
    // Returns string type for a service object
    return 'alicia_flexible_grasp_supervisor/TcpCalibrationCommandResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '024a38e797fbdb70356ab0fa543ee5f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    uint32 sample_count
    geometry_msgs/Vector3 tcp_translation
    geometry_msgs/Vector3 fixed_point
    float64 rms_error_m
    float64 max_error_m
    float64 orientation_span_deg
    string result_file
    
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TcpCalibrationCommandResponse(null);
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

    if (msg.sample_count !== undefined) {
      resolved.sample_count = msg.sample_count;
    }
    else {
      resolved.sample_count = 0
    }

    if (msg.tcp_translation !== undefined) {
      resolved.tcp_translation = geometry_msgs.msg.Vector3.Resolve(msg.tcp_translation)
    }
    else {
      resolved.tcp_translation = new geometry_msgs.msg.Vector3()
    }

    if (msg.fixed_point !== undefined) {
      resolved.fixed_point = geometry_msgs.msg.Vector3.Resolve(msg.fixed_point)
    }
    else {
      resolved.fixed_point = new geometry_msgs.msg.Vector3()
    }

    if (msg.rms_error_m !== undefined) {
      resolved.rms_error_m = msg.rms_error_m;
    }
    else {
      resolved.rms_error_m = 0.0
    }

    if (msg.max_error_m !== undefined) {
      resolved.max_error_m = msg.max_error_m;
    }
    else {
      resolved.max_error_m = 0.0
    }

    if (msg.orientation_span_deg !== undefined) {
      resolved.orientation_span_deg = msg.orientation_span_deg;
    }
    else {
      resolved.orientation_span_deg = 0.0
    }

    if (msg.result_file !== undefined) {
      resolved.result_file = msg.result_file;
    }
    else {
      resolved.result_file = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: TcpCalibrationCommandRequest,
  Response: TcpCalibrationCommandResponse,
  md5sum() { return '58e49899f0eda63c395f3d3908079771'; },
  datatype() { return 'alicia_flexible_grasp_supervisor/TcpCalibrationCommand'; }
};
