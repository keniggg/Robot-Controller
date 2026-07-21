// Auto-generated. Do not edit!

// (in-package alicia_flexible_grasp_supervisor.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ObjectGeometry = require('./ObjectGeometry.js');
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Grasp6DPlan {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.valid = null;
      this.poses = null;
      this.score = null;
      this.candidate_width_m = null;
      this.required_open_width_m = null;
      this.object_geometry = null;
      this.model_choice = null;
      this.plan_id = null;
      this.diagnostic = null;
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
      if (initObj.hasOwnProperty('poses')) {
        this.poses = initObj.poses
      }
      else {
        this.poses = [];
      }
      if (initObj.hasOwnProperty('score')) {
        this.score = initObj.score
      }
      else {
        this.score = 0.0;
      }
      if (initObj.hasOwnProperty('candidate_width_m')) {
        this.candidate_width_m = initObj.candidate_width_m
      }
      else {
        this.candidate_width_m = 0.0;
      }
      if (initObj.hasOwnProperty('required_open_width_m')) {
        this.required_open_width_m = initObj.required_open_width_m
      }
      else {
        this.required_open_width_m = 0.0;
      }
      if (initObj.hasOwnProperty('object_geometry')) {
        this.object_geometry = initObj.object_geometry
      }
      else {
        this.object_geometry = new ObjectGeometry();
      }
      if (initObj.hasOwnProperty('model_choice')) {
        this.model_choice = initObj.model_choice
      }
      else {
        this.model_choice = '';
      }
      if (initObj.hasOwnProperty('plan_id')) {
        this.plan_id = initObj.plan_id
      }
      else {
        this.plan_id = '';
      }
      if (initObj.hasOwnProperty('diagnostic')) {
        this.diagnostic = initObj.diagnostic
      }
      else {
        this.diagnostic = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Grasp6DPlan
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [valid]
    bufferOffset = _serializer.bool(obj.valid, buffer, bufferOffset);
    // Serialize message field [poses]
    // Serialize the length for message field [poses]
    bufferOffset = _serializer.uint32(obj.poses.length, buffer, bufferOffset);
    obj.poses.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [score]
    bufferOffset = _serializer.float32(obj.score, buffer, bufferOffset);
    // Serialize message field [candidate_width_m]
    bufferOffset = _serializer.float32(obj.candidate_width_m, buffer, bufferOffset);
    // Serialize message field [required_open_width_m]
    bufferOffset = _serializer.float32(obj.required_open_width_m, buffer, bufferOffset);
    // Serialize message field [object_geometry]
    bufferOffset = ObjectGeometry.serialize(obj.object_geometry, buffer, bufferOffset);
    // Serialize message field [model_choice]
    bufferOffset = _serializer.string(obj.model_choice, buffer, bufferOffset);
    // Serialize message field [plan_id]
    bufferOffset = _serializer.string(obj.plan_id, buffer, bufferOffset);
    // Serialize message field [diagnostic]
    bufferOffset = _serializer.string(obj.diagnostic, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Grasp6DPlan
    let len;
    let data = new Grasp6DPlan(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [valid]
    data.valid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [poses]
    // Deserialize array length for message field [poses]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poses = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poses[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [score]
    data.score = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [candidate_width_m]
    data.candidate_width_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [required_open_width_m]
    data.required_open_width_m = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [object_geometry]
    data.object_geometry = ObjectGeometry.deserialize(buffer, bufferOffset);
    // Deserialize message field [model_choice]
    data.model_choice = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [plan_id]
    data.plan_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [diagnostic]
    data.diagnostic = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 56 * object.poses.length;
    length += ObjectGeometry.getMessageSize(object.object_geometry);
    length += _getByteLength(object.model_choice);
    length += _getByteLength(object.plan_id);
    length += _getByteLength(object.diagnostic);
    return length + 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/Grasp6DPlan';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a90c722154d42b08b0800cd88d52ff9d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool valid
    geometry_msgs/Pose[] poses
    float32 score
    float32 candidate_width_m
    float32 required_open_width_m
    alicia_flexible_grasp_supervisor/ObjectGeometry object_geometry
    string model_choice
    string plan_id
    string diagnostic
    
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
    MSG: alicia_flexible_grasp_supervisor/ObjectGeometry
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
    const resolved = new Grasp6DPlan(null);
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

    if (msg.poses !== undefined) {
      resolved.poses = new Array(msg.poses.length);
      for (let i = 0; i < resolved.poses.length; ++i) {
        resolved.poses[i] = geometry_msgs.msg.Pose.Resolve(msg.poses[i]);
      }
    }
    else {
      resolved.poses = []
    }

    if (msg.score !== undefined) {
      resolved.score = msg.score;
    }
    else {
      resolved.score = 0.0
    }

    if (msg.candidate_width_m !== undefined) {
      resolved.candidate_width_m = msg.candidate_width_m;
    }
    else {
      resolved.candidate_width_m = 0.0
    }

    if (msg.required_open_width_m !== undefined) {
      resolved.required_open_width_m = msg.required_open_width_m;
    }
    else {
      resolved.required_open_width_m = 0.0
    }

    if (msg.object_geometry !== undefined) {
      resolved.object_geometry = ObjectGeometry.Resolve(msg.object_geometry)
    }
    else {
      resolved.object_geometry = new ObjectGeometry()
    }

    if (msg.model_choice !== undefined) {
      resolved.model_choice = msg.model_choice;
    }
    else {
      resolved.model_choice = ''
    }

    if (msg.plan_id !== undefined) {
      resolved.plan_id = msg.plan_id;
    }
    else {
      resolved.plan_id = ''
    }

    if (msg.diagnostic !== undefined) {
      resolved.diagnostic = msg.diagnostic;
    }
    else {
      resolved.diagnostic = ''
    }

    return resolved;
    }
};

module.exports = Grasp6DPlan;
