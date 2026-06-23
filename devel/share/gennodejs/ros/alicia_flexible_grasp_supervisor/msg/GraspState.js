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

class GraspState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state = null;
      this.state_name = null;
      this.running = null;
      this.success = null;
      this.message = null;
      this.current_force = null;
      this.target_force = null;
      this.object_pose_base = null;
      this.target_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = 0;
      }
      if (initObj.hasOwnProperty('state_name')) {
        this.state_name = initObj.state_name
      }
      else {
        this.state_name = '';
      }
      if (initObj.hasOwnProperty('running')) {
        this.running = initObj.running
      }
      else {
        this.running = false;
      }
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
      if (initObj.hasOwnProperty('current_force')) {
        this.current_force = initObj.current_force
      }
      else {
        this.current_force = 0.0;
      }
      if (initObj.hasOwnProperty('target_force')) {
        this.target_force = initObj.target_force
      }
      else {
        this.target_force = 0.0;
      }
      if (initObj.hasOwnProperty('object_pose_base')) {
        this.object_pose_base = initObj.object_pose_base
      }
      else {
        this.object_pose_base = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('target_pose')) {
        this.target_pose = initObj.target_pose
      }
      else {
        this.target_pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GraspState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.uint8(obj.state, buffer, bufferOffset);
    // Serialize message field [state_name]
    bufferOffset = _serializer.string(obj.state_name, buffer, bufferOffset);
    // Serialize message field [running]
    bufferOffset = _serializer.bool(obj.running, buffer, bufferOffset);
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [current_force]
    bufferOffset = _serializer.float32(obj.current_force, buffer, bufferOffset);
    // Serialize message field [target_force]
    bufferOffset = _serializer.float32(obj.target_force, buffer, bufferOffset);
    // Serialize message field [object_pose_base]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.object_pose_base, buffer, bufferOffset);
    // Serialize message field [target_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.target_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GraspState
    let len;
    let data = new GraspState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [state_name]
    data.state_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [running]
    data.running = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [current_force]
    data.current_force = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [target_force]
    data.target_force = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [object_pose_base]
    data.object_pose_base = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [target_pose]
    data.target_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.state_name);
    length += _getByteLength(object.message);
    return length + 131;
  }

  static datatype() {
    // Returns string type for a message object
    return 'alicia_flexible_grasp_supervisor/GraspState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75215c45032076e51dbd82164d6951cb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    uint8 IDLE=0
    uint8 SEARCH_OBJECT=1
    uint8 ESTIMATE_POSE=2
    uint8 PLAN_PREGRASP=3
    uint8 MOVE_PREGRASP=4
    uint8 APPROACH_OBJECT=5
    uint8 COMPLIANT_CLOSE=6
    uint8 GRASP_VERIFY=7
    uint8 LIFT_OBJECT=8
    uint8 PLACE_OBJECT=9
    uint8 RELEASE_OBJECT=10
    uint8 SUCCESS=11
    uint8 FAILED=12
    uint8 EMERGENCY_STOP=13
    uint8 state
    string state_name
    bool running
    bool success
    string message
    float32 current_force
    float32 target_force
    geometry_msgs/Pose object_pose_base
    geometry_msgs/Pose target_pose
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GraspState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = 0
    }

    if (msg.state_name !== undefined) {
      resolved.state_name = msg.state_name;
    }
    else {
      resolved.state_name = ''
    }

    if (msg.running !== undefined) {
      resolved.running = msg.running;
    }
    else {
      resolved.running = false
    }

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

    if (msg.current_force !== undefined) {
      resolved.current_force = msg.current_force;
    }
    else {
      resolved.current_force = 0.0
    }

    if (msg.target_force !== undefined) {
      resolved.target_force = msg.target_force;
    }
    else {
      resolved.target_force = 0.0
    }

    if (msg.object_pose_base !== undefined) {
      resolved.object_pose_base = geometry_msgs.msg.Pose.Resolve(msg.object_pose_base)
    }
    else {
      resolved.object_pose_base = new geometry_msgs.msg.Pose()
    }

    if (msg.target_pose !== undefined) {
      resolved.target_pose = geometry_msgs.msg.Pose.Resolve(msg.target_pose)
    }
    else {
      resolved.target_pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

// Constants for message
GraspState.Constants = {
  IDLE: 0,
  SEARCH_OBJECT: 1,
  ESTIMATE_POSE: 2,
  PLAN_PREGRASP: 3,
  MOVE_PREGRASP: 4,
  APPROACH_OBJECT: 5,
  COMPLIANT_CLOSE: 6,
  GRASP_VERIFY: 7,
  LIFT_OBJECT: 8,
  PLACE_OBJECT: 9,
  RELEASE_OBJECT: 10,
  SUCCESS: 11,
  FAILED: 12,
  EMERGENCY_STOP: 13,
}

module.exports = GraspState;
