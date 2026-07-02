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
      this.bbox_x = null;
      this.bbox_y = null;
      this.bbox_width = null;
      this.bbox_height = null;
      this.depth_m = null;
      this.pose_camera = null;
      this.pose_base = null;
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
      if (initObj.hasOwnProperty('bbox_x')) {
        this.bbox_x = initObj.bbox_x
      }
      else {
        this.bbox_x = 0;
      }
      if (initObj.hasOwnProperty('bbox_y')) {
        this.bbox_y = initObj.bbox_y
      }
      else {
        this.bbox_y = 0;
      }
      if (initObj.hasOwnProperty('bbox_width')) {
        this.bbox_width = initObj.bbox_width
      }
      else {
        this.bbox_width = 0;
      }
      if (initObj.hasOwnProperty('bbox_height')) {
        this.bbox_height = initObj.bbox_height
      }
      else {
        this.bbox_height = 0;
      }
      if (initObj.hasOwnProperty('depth_m')) {
        this.depth_m = initObj.depth_m
      }
      else {
        this.depth_m = 0.0;
      }
      if (initObj.hasOwnProperty('pose_camera')) {
        this.pose_camera = initObj.pose_camera
      }
      else {
        this.pose_camera = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('pose_base')) {
        this.pose_base = initObj.pose_base
      }
      else {
        this.pose_base = new geometry_msgs.msg.PoseStamped();
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
    // Serialize message field [bbox_x]
    bufferOffset = _serializer.uint32(obj.bbox_x, buffer, bufferOffset);
    // Serialize message field [bbox_y]
    bufferOffset = _serializer.uint32(obj.bbox_y, buffer, bufferOffset);
    // Serialize message field [bbox_width]
    bufferOffset = _serializer.uint32(obj.bbox_width, buffer, bufferOffset);
    // Serialize message field [bbox_height]
    bufferOffset = _serializer.uint32(obj.bbox_height, buffer, bufferOffset);
    // Serialize message field [depth_m]
    bufferOffset = _serializer.float32(obj.depth_m, buffer, bufferOffset);
    // Serialize message field [pose_camera]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.pose_camera, buffer, bufferOffset);
    // Serialize message field [pose_base]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.pose_base, buffer, bufferOffset);
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
    // Deserialize message field [bbox_x]
    data.bbox_x = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bbox_y]
    data.bbox_y = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bbox_width]
    data.bbox_width = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bbox_height]
    data.bbox_height = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [depth_m]
    data.depth_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pose_camera]
    data.pose_camera = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose_base]
    data.pose_base = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.label);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.pose_camera);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.pose_base);
    return length + 37;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/ObjectPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d3731316703d38acb9f83ae0dd46e90';
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
    uint32 bbox_x
    uint32 bbox_y
    uint32 bbox_width
    uint32 bbox_height
    float32 depth_m
    geometry_msgs/PoseStamped pose_camera
    geometry_msgs/PoseStamped pose_base
    
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
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
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

    if (msg.bbox_x !== undefined) {
      resolved.bbox_x = msg.bbox_x;
    }
    else {
      resolved.bbox_x = 0
    }

    if (msg.bbox_y !== undefined) {
      resolved.bbox_y = msg.bbox_y;
    }
    else {
      resolved.bbox_y = 0
    }

    if (msg.bbox_width !== undefined) {
      resolved.bbox_width = msg.bbox_width;
    }
    else {
      resolved.bbox_width = 0
    }

    if (msg.bbox_height !== undefined) {
      resolved.bbox_height = msg.bbox_height;
    }
    else {
      resolved.bbox_height = 0
    }

    if (msg.depth_m !== undefined) {
      resolved.depth_m = msg.depth_m;
    }
    else {
      resolved.depth_m = 0.0
    }

    if (msg.pose_camera !== undefined) {
      resolved.pose_camera = geometry_msgs.msg.PoseStamped.Resolve(msg.pose_camera)
    }
    else {
      resolved.pose_camera = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.pose_base !== undefined) {
      resolved.pose_base = geometry_msgs.msg.PoseStamped.Resolve(msg.pose_base)
    }
    else {
      resolved.pose_base = new geometry_msgs.msg.PoseStamped()
    }

    return resolved;
    }
};

module.exports = ObjectPose;
