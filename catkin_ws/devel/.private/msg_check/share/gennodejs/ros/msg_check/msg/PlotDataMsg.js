// Auto-generated. Do not edit!

// (in-package msg_check.msg)


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

class PlotDataMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.curpos = null;
      this.despos = null;
      this.curvel = null;
      this.desvel = null;
      this.curacc = null;
      this.desacc = null;
      this.poserr = null;
      this.velerr = null;
      this.curor = null;
      this.desor = null;
      this.thrust = null;
      this.M1_pos_err = null;
      this.M2_pos_err = null;
      this.M1_vel_err = null;
      this.M2_vel_err = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('curpos')) {
        this.curpos = initObj.curpos
      }
      else {
        this.curpos = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('despos')) {
        this.despos = initObj.despos
      }
      else {
        this.despos = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('curvel')) {
        this.curvel = initObj.curvel
      }
      else {
        this.curvel = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('desvel')) {
        this.desvel = initObj.desvel
      }
      else {
        this.desvel = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('curacc')) {
        this.curacc = initObj.curacc
      }
      else {
        this.curacc = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('desacc')) {
        this.desacc = initObj.desacc
      }
      else {
        this.desacc = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('poserr')) {
        this.poserr = initObj.poserr
      }
      else {
        this.poserr = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velerr')) {
        this.velerr = initObj.velerr
      }
      else {
        this.velerr = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('curor')) {
        this.curor = initObj.curor
      }
      else {
        this.curor = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('desor')) {
        this.desor = initObj.desor
      }
      else {
        this.desor = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0.0;
      }
      if (initObj.hasOwnProperty('M1_pos_err')) {
        this.M1_pos_err = initObj.M1_pos_err
      }
      else {
        this.M1_pos_err = 0.0;
      }
      if (initObj.hasOwnProperty('M2_pos_err')) {
        this.M2_pos_err = initObj.M2_pos_err
      }
      else {
        this.M2_pos_err = 0.0;
      }
      if (initObj.hasOwnProperty('M1_vel_err')) {
        this.M1_vel_err = initObj.M1_vel_err
      }
      else {
        this.M1_vel_err = 0.0;
      }
      if (initObj.hasOwnProperty('M2_vel_err')) {
        this.M2_vel_err = initObj.M2_vel_err
      }
      else {
        this.M2_vel_err = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlotDataMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [curpos]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.curpos, buffer, bufferOffset);
    // Serialize message field [despos]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.despos, buffer, bufferOffset);
    // Serialize message field [curvel]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.curvel, buffer, bufferOffset);
    // Serialize message field [desvel]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.desvel, buffer, bufferOffset);
    // Serialize message field [curacc]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.curacc, buffer, bufferOffset);
    // Serialize message field [desacc]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.desacc, buffer, bufferOffset);
    // Serialize message field [poserr]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.poserr, buffer, bufferOffset);
    // Serialize message field [velerr]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velerr, buffer, bufferOffset);
    // Serialize message field [curor]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.curor, buffer, bufferOffset);
    // Serialize message field [desor]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.desor, buffer, bufferOffset);
    // Serialize message field [thrust]
    bufferOffset = _serializer.float64(obj.thrust, buffer, bufferOffset);
    // Serialize message field [M1_pos_err]
    bufferOffset = _serializer.float64(obj.M1_pos_err, buffer, bufferOffset);
    // Serialize message field [M2_pos_err]
    bufferOffset = _serializer.float64(obj.M2_pos_err, buffer, bufferOffset);
    // Serialize message field [M1_vel_err]
    bufferOffset = _serializer.float64(obj.M1_vel_err, buffer, bufferOffset);
    // Serialize message field [M2_vel_err]
    bufferOffset = _serializer.float64(obj.M2_vel_err, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlotDataMsg
    let len;
    let data = new PlotDataMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [curpos]
    data.curpos = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [despos]
    data.despos = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [curvel]
    data.curvel = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [desvel]
    data.desvel = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [curacc]
    data.curacc = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [desacc]
    data.desacc = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [poserr]
    data.poserr = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velerr]
    data.velerr = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [curor]
    data.curor = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [desor]
    data.desor = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [M1_pos_err]
    data.M1_pos_err = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [M2_pos_err]
    data.M2_pos_err = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [M1_vel_err]
    data.M1_vel_err = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [M2_vel_err]
    data.M2_vel_err = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 296;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msg_check/PlotDataMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d96fc0f4d709ad5a1e1b4d1fe446936';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    geometry_msgs/Vector3 curpos
    geometry_msgs/Vector3 despos
    
    geometry_msgs/Vector3 curvel
    geometry_msgs/Vector3 desvel
    
    geometry_msgs/Vector3 curacc
    geometry_msgs/Vector3 desacc
    
    geometry_msgs/Vector3 poserr
    geometry_msgs/Vector3 velerr
    
    geometry_msgs/Quaternion curor
    geometry_msgs/Quaternion desor
    
    float64 thrust
    float64 M1_pos_err
    float64 M2_pos_err
    float64 M1_vel_err
    float64 M2_vel_err
    
    
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
    const resolved = new PlotDataMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.curpos !== undefined) {
      resolved.curpos = geometry_msgs.msg.Vector3.Resolve(msg.curpos)
    }
    else {
      resolved.curpos = new geometry_msgs.msg.Vector3()
    }

    if (msg.despos !== undefined) {
      resolved.despos = geometry_msgs.msg.Vector3.Resolve(msg.despos)
    }
    else {
      resolved.despos = new geometry_msgs.msg.Vector3()
    }

    if (msg.curvel !== undefined) {
      resolved.curvel = geometry_msgs.msg.Vector3.Resolve(msg.curvel)
    }
    else {
      resolved.curvel = new geometry_msgs.msg.Vector3()
    }

    if (msg.desvel !== undefined) {
      resolved.desvel = geometry_msgs.msg.Vector3.Resolve(msg.desvel)
    }
    else {
      resolved.desvel = new geometry_msgs.msg.Vector3()
    }

    if (msg.curacc !== undefined) {
      resolved.curacc = geometry_msgs.msg.Vector3.Resolve(msg.curacc)
    }
    else {
      resolved.curacc = new geometry_msgs.msg.Vector3()
    }

    if (msg.desacc !== undefined) {
      resolved.desacc = geometry_msgs.msg.Vector3.Resolve(msg.desacc)
    }
    else {
      resolved.desacc = new geometry_msgs.msg.Vector3()
    }

    if (msg.poserr !== undefined) {
      resolved.poserr = geometry_msgs.msg.Vector3.Resolve(msg.poserr)
    }
    else {
      resolved.poserr = new geometry_msgs.msg.Vector3()
    }

    if (msg.velerr !== undefined) {
      resolved.velerr = geometry_msgs.msg.Vector3.Resolve(msg.velerr)
    }
    else {
      resolved.velerr = new geometry_msgs.msg.Vector3()
    }

    if (msg.curor !== undefined) {
      resolved.curor = geometry_msgs.msg.Quaternion.Resolve(msg.curor)
    }
    else {
      resolved.curor = new geometry_msgs.msg.Quaternion()
    }

    if (msg.desor !== undefined) {
      resolved.desor = geometry_msgs.msg.Quaternion.Resolve(msg.desor)
    }
    else {
      resolved.desor = new geometry_msgs.msg.Quaternion()
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0.0
    }

    if (msg.M1_pos_err !== undefined) {
      resolved.M1_pos_err = msg.M1_pos_err;
    }
    else {
      resolved.M1_pos_err = 0.0
    }

    if (msg.M2_pos_err !== undefined) {
      resolved.M2_pos_err = msg.M2_pos_err;
    }
    else {
      resolved.M2_pos_err = 0.0
    }

    if (msg.M1_vel_err !== undefined) {
      resolved.M1_vel_err = msg.M1_vel_err;
    }
    else {
      resolved.M1_vel_err = 0.0
    }

    if (msg.M2_vel_err !== undefined) {
      resolved.M2_vel_err = msg.M2_vel_err;
    }
    else {
      resolved.M2_vel_err = 0.0
    }

    return resolved;
    }
};

module.exports = PlotDataMsg;
