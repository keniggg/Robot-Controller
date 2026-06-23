// Auto-generated. Do not edit!

// (in-package alicia_duo_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ArmJointState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint1 = null;
      this.joint2 = null;
      this.joint3 = null;
      this.joint4 = null;
      this.joint5 = null;
      this.joint6 = null;
      this.gripper = null;
      this.time = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint1')) {
        this.joint1 = initObj.joint1
      }
      else {
        this.joint1 = 0.0;
      }
      if (initObj.hasOwnProperty('joint2')) {
        this.joint2 = initObj.joint2
      }
      else {
        this.joint2 = 0.0;
      }
      if (initObj.hasOwnProperty('joint3')) {
        this.joint3 = initObj.joint3
      }
      else {
        this.joint3 = 0.0;
      }
      if (initObj.hasOwnProperty('joint4')) {
        this.joint4 = initObj.joint4
      }
      else {
        this.joint4 = 0.0;
      }
      if (initObj.hasOwnProperty('joint5')) {
        this.joint5 = initObj.joint5
      }
      else {
        this.joint5 = 0.0;
      }
      if (initObj.hasOwnProperty('joint6')) {
        this.joint6 = initObj.joint6
      }
      else {
        this.joint6 = 0.0;
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0.0;
      }
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ArmJointState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [joint1]
    bufferOffset = _serializer.float32(obj.joint1, buffer, bufferOffset);
    // Serialize message field [joint2]
    bufferOffset = _serializer.float32(obj.joint2, buffer, bufferOffset);
    // Serialize message field [joint3]
    bufferOffset = _serializer.float32(obj.joint3, buffer, bufferOffset);
    // Serialize message field [joint4]
    bufferOffset = _serializer.float32(obj.joint4, buffer, bufferOffset);
    // Serialize message field [joint5]
    bufferOffset = _serializer.float32(obj.joint5, buffer, bufferOffset);
    // Serialize message field [joint6]
    bufferOffset = _serializer.float32(obj.joint6, buffer, bufferOffset);
    // Serialize message field [gripper]
    bufferOffset = _serializer.float32(obj.gripper, buffer, bufferOffset);
    // Serialize message field [time]
    bufferOffset = _serializer.float32(obj.time, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ArmJointState
    let len;
    let data = new ArmJointState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint1]
    data.joint1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [joint2]
    data.joint2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [joint3]
    data.joint3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [joint4]
    data.joint4 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [joint5]
    data.joint5 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [joint6]
    data.joint6 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [time]
    data.time = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_duo_driver/ArmJointState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9825570808a8e3729693705ffcdd81b9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 标准机械臂关节状态消息 (弧度单位)
    # 所有关节角度使用弧度作为单位
    
    Header header
    
    # 六个主要关节角度 (弧度)
    float32 joint1
    float32 joint2
    float32 joint3
    float32 joint4
    float32 joint5
    float32 joint6
    
    # 夹爪角度 (弧度)
    float32 gripper
    
    # 可选的运动控制参数
    float32 time  # 运动时间(秒)，默认为0表示立即执行
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
    const resolved = new ArmJointState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint1 !== undefined) {
      resolved.joint1 = msg.joint1;
    }
    else {
      resolved.joint1 = 0.0
    }

    if (msg.joint2 !== undefined) {
      resolved.joint2 = msg.joint2;
    }
    else {
      resolved.joint2 = 0.0
    }

    if (msg.joint3 !== undefined) {
      resolved.joint3 = msg.joint3;
    }
    else {
      resolved.joint3 = 0.0
    }

    if (msg.joint4 !== undefined) {
      resolved.joint4 = msg.joint4;
    }
    else {
      resolved.joint4 = 0.0
    }

    if (msg.joint5 !== undefined) {
      resolved.joint5 = msg.joint5;
    }
    else {
      resolved.joint5 = 0.0
    }

    if (msg.joint6 !== undefined) {
      resolved.joint6 = msg.joint6;
    }
    else {
      resolved.joint6 = 0.0
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0.0
    }

    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    return resolved;
    }
};

module.exports = ArmJointState;
