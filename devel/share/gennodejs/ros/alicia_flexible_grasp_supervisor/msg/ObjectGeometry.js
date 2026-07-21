// Auto-generated. Do not edit!

// (in-package alicia_flexible_grasp_supervisor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ObjectGeometry {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.valid = null;
      this.label = null;
      this.source_mode = null;
      this.pose_base = null;
      this.size_xyz_m = null;
      this.support_normal_base = null;
      this.support_offset_m = null;
      this.valid_depth_points = null;
      this.valid_depth_ratio = null;
      this.depth_mad_m = null;
      this.fused_frames = null;
      this.support_inlier_ratio = null;
      this.object_point_count = null;
      this.failure_reason = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('valid')) {
        this.valid = initObj.valid
      }
      else {
        this.valid = false;
      }
      if (initObj.hasOwnProperty('label')) {
        this.label = initObj.label
      }
      else {
        this.label = '';
      }
      if (initObj.hasOwnProperty('source_mode')) {
        this.source_mode = initObj.source_mode
      }
      else {
        this.source_mode = '';
      }
      if (initObj.hasOwnProperty('pose_base')) {
        this.pose_base = initObj.pose_base
      }
      else {
        this.pose_base = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('size_xyz_m')) {
        this.size_xyz_m = initObj.size_xyz_m
      }
      else {
        this.size_xyz_m = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('support_normal_base')) {
        this.support_normal_base = initObj.support_normal_base
      }
      else {
        this.support_normal_base = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('support_offset_m')) {
        this.support_offset_m = initObj.support_offset_m
      }
      else {
        this.support_offset_m = 0.0;
      }
      if (initObj.hasOwnProperty('valid_depth_points')) {
        this.valid_depth_points = initObj.valid_depth_points
      }
      else {
        this.valid_depth_points = 0;
      }
      if (initObj.hasOwnProperty('valid_depth_ratio')) {
        this.valid_depth_ratio = initObj.valid_depth_ratio
      }
      else {
        this.valid_depth_ratio = 0.0;
      }
      if (initObj.hasOwnProperty('depth_mad_m')) {
        this.depth_mad_m = initObj.depth_mad_m
      }
      else {
        this.depth_mad_m = 0.0;
      }
      if (initObj.hasOwnProperty('fused_frames')) {
        this.fused_frames = initObj.fused_frames
      }
      else {
        this.fused_frames = 0;
      }
      if (initObj.hasOwnProperty('support_inlier_ratio')) {
        this.support_inlier_ratio = initObj.support_inlier_ratio
      }
      else {
        this.support_inlier_ratio = 0.0;
      }
      if (initObj.hasOwnProperty('object_point_count')) {
        this.object_point_count = initObj.object_point_count
      }
      else {
        this.object_point_count = 0;
      }
      if (initObj.hasOwnProperty('failure_reason')) {
        this.failure_reason = initObj.failure_reason
      }
      else {
        this.failure_reason = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectGeometry
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    // Serialize message field [label]
    bufferOffset = _serializer.string(obj.label, buffer, bufferOffset);
    // Serialize message field [source_mode]
    bufferOffset = _serializer.string(obj.source_mode, buffer, bufferOffset);
    // Serialize message field [pose_base]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose_base, buffer, bufferOffset);
    // Serialize message field [size_xyz_m]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.size_xyz_m, buffer, bufferOffset);
    // Serialize message field [support_normal_base]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.support_normal_base, buffer, bufferOffset);
    // Serialize message field [support_offset_m]
    bufferOffset = _serializer.float32(obj.support_offset_m, buffer, bufferOffset);
    // Serialize message field [valid_depth_points]
    bufferOffset = _serializer.uint32(obj.valid_depth_points, buffer, bufferOffset);
    // Serialize message field [valid_depth_ratio]
    bufferOffset = _serializer.float32(obj.valid_depth_ratio, buffer, bufferOffset);
    // Serialize message field [depth_mad_m]
    bufferOffset = _serializer.float32(obj.depth_mad_m, buffer, bufferOffset);
    // Serialize message field [fused_frames]
    bufferOffset = _serializer.uint32(obj.fused_frames, buffer, bufferOffset);
    // Serialize message field [support_inlier_ratio]
    bufferOffset = _serializer.float32(obj.support_inlier_ratio, buffer, bufferOffset);
    // Serialize message field [object_point_count]
    bufferOffset = _serializer.uint32(obj.object_point_count, buffer, bufferOffset);
    // Serialize message field [failure_reason]
    bufferOffset = _serializer.string(obj.failure_reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectGeometry
    let len;
    let data = new ObjectGeometry(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [label]
    data.label = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [source_mode]
    data.source_mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pose_base]
    data.pose_base = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [size_xyz_m]
    data.size_xyz_m = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [support_normal_base]
    data.support_normal_base = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [support_offset_m]
    data.support_offset_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [valid_depth_points]
    data.valid_depth_points = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [valid_depth_ratio]
    data.valid_depth_ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [depth_mad_m]
    data.depth_mad_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [fused_frames]
    data.fused_frames = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [support_inlier_ratio]
    data.support_inlier_ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [object_point_count]
    data.object_point_count = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [failure_reason]
    data.failure_reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.label);
    length += _getByteLength(object.source_mode);
    length += _getByteLength(object.failure_reason);
    return length + 145;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/ObjectGeometry';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be6e291b8f028ecb31c9e9f8f891dc32';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool valid
    string label
    string source_mode
    geometry_msgs/Pose pose_base
    geometry_msgs/Vector3 size_xyz_m
    geometry_msgs/Vector3 support_normal_base
    float32 support_offset_m
    uint32 valid_depth_points
    float32 valid_depth_ratio
    float32 depth_mad_m
    uint32 fused_frames
    float32 support_inlier_ratio
    uint32 object_point_count
    string failure_reason
    
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
    const resolved = new ObjectGeometry(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.valid !== undefined) {
      resolved.valid = msg.valid;
    }
    else {
      resolved.valid = false
    }

    if (msg.label !== undefined) {
      resolved.label = msg.label;
    }
    else {
      resolved.label = ''
    }

    if (msg.source_mode !== undefined) {
      resolved.source_mode = msg.source_mode;
    }
    else {
      resolved.source_mode = ''
    }

    if (msg.pose_base !== undefined) {
      resolved.pose_base = geometry_msgs.msg.Pose.Resolve(msg.pose_base)
    }
    else {
      resolved.pose_base = new geometry_msgs.msg.Pose()
    }

    if (msg.size_xyz_m !== undefined) {
      resolved.size_xyz_m = geometry_msgs.msg.Vector3.Resolve(msg.size_xyz_m)
    }
    else {
      resolved.size_xyz_m = new geometry_msgs.msg.Vector3()
    }

    if (msg.support_normal_base !== undefined) {
      resolved.support_normal_base = geometry_msgs.msg.Vector3.Resolve(msg.support_normal_base)
    }
    else {
      resolved.support_normal_base = new geometry_msgs.msg.Vector3()
    }

    if (msg.support_offset_m !== undefined) {
      resolved.support_offset_m = msg.support_offset_m;
    }
    else {
      resolved.support_offset_m = 0.0
    }

    if (msg.valid_depth_points !== undefined) {
      resolved.valid_depth_points = msg.valid_depth_points;
    }
    else {
      resolved.valid_depth_points = 0
    }

    if (msg.valid_depth_ratio !== undefined) {
      resolved.valid_depth_ratio = msg.valid_depth_ratio;
    }
    else {
      resolved.valid_depth_ratio = 0.0
    }

    if (msg.depth_mad_m !== undefined) {
      resolved.depth_mad_m = msg.depth_mad_m;
    }
    else {
      resolved.depth_mad_m = 0.0
    }

    if (msg.fused_frames !== undefined) {
      resolved.fused_frames = msg.fused_frames;
    }
    else {
      resolved.fused_frames = 0
    }

    if (msg.support_inlier_ratio !== undefined) {
      resolved.support_inlier_ratio = msg.support_inlier_ratio;
    }
    else {
      resolved.support_inlier_ratio = 0.0
    }

    if (msg.object_point_count !== undefined) {
      resolved.object_point_count = msg.object_point_count;
    }
    else {
      resolved.object_point_count = 0
    }

    if (msg.failure_reason !== undefined) {
      resolved.failure_reason = msg.failure_reason;
    }
    else {
      resolved.failure_reason = ''
    }

    return resolved;
    }
};

module.exports = ObjectGeometry;
