// Auto-generated. Do not edit!

// (in-package alicia_flexible_grasp_supervisor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObjectPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detected = null;
      this.label = null;
      this.confidence = null;
      this.u = null;
      this.v = null;
      this.depth = null;
      this.position_camera = null;
      this.pose_base = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('detected')) {
        this.detected = initObj.detected
      }
      else {
        this.detected = false;
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('u')) {
        this.u = initObj.u
      }
      else {
        this.u = 0;
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = 0;
      }
      if (initObj.hasOwnProperty('depth')) {
        this.depth = initObj.depth
      }
      else {
        this.depth = 0.0;
      }
      if (initObj.hasOwnProperty('position_camera')) {
        this.position_camera = initObj.position_camera
      }
      else {
        this.position_camera = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('pose_base')) {
        this.pose_base = initObj.pose_base
      }
      else {
        this.pose_base = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectPose
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detected]
    bufferOffset = _serializer.bool(obj.detected, buffer, bufferOffset);
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [u]
    bufferOffset = _serializer.uint32(obj.u, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = _serializer.uint32(obj.v, buffer, bufferOffset);
    // Serialize message field [depth]
    bufferOffset = _serializer.float32(obj.depth, buffer, bufferOffset);
    // Serialize message field [position_camera]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position_camera, buffer, bufferOffset);
    // Serialize message field [pose_base]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose_base, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectPose
    let len;
    let data = new ObjectPose(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detected]
    data.detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [u]
    data.u = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [depth]
    data.depth = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [position_camera]
    data.position_camera = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose_base]
    data.pose_base = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.label);
    length += _getByteLength(object.status);
    return length + 105;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/ObjectPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '613afc19135cb8f109a247ba2e6628c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool detected
    string label
    float32 confidence
    uint32 u
    uint32 v
    float32 depth
    geometry_msgs/Point position_camera
    geometry_msgs/Pose pose_base
    string status
    
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
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObjectPose(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.detected !== undefined) {
      resolved.detected = msg.detected;
    }
    else {
      resolved.detected = false
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.u !== undefined) {
      resolved.u = msg.u;
    }
    else {
      resolved.u = 0
    }

    if (msg.v !== undefined) {
      resolved.v = msg.v;
    }
    else {
      resolved.v = 0
    }

    if (msg.depth !== undefined) {
      resolved.depth = msg.depth;
    }
    else {
      resolved.depth = 0.0
    }

    if (msg.position_camera !== undefined) {
      resolved.position_camera = geometry_msgs.msg.Point.Resolve(msg.position_camera)
    }
    else {
      resolved.position_camera = new geometry_msgs.msg.Point()
    }

    if (msg.pose_base !== undefined) {
      resolved.pose_base = geometry_msgs.msg.Pose.Resolve(msg.pose_base)
    }
    else {
      resolved.pose_base = new geometry_msgs.msg.Pose()
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    return resolved;
    }
};

module.exports = ObjectPose;
