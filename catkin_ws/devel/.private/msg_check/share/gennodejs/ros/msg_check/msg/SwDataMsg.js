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

class SwDataMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.sp = null;
      this.sq = null;
      this.acceleration = null;
      this.angular_acceleration = null;
      this.position_error = null;
      this.velocity_error = null;
      this.position_error_integral = null;
      this.angle_error = null;
      this.angle_rate_error = null;
      this.Kp0_1 = null;
      this.Kp1_1 = null;
      this.Kp0_2 = null;
      this.Kp1_2 = null;
      this.Kp0_3 = null;
      this.Kp1_3 = null;
      this.Kq0_1 = null;
      this.Kq1_1 = null;
      this.Kq2_1 = null;
      this.Kq0_2 = null;
      this.Kq1_2 = null;
      this.Kq2_2 = null;
      this.Kq0_3 = null;
      this.Kq1_3 = null;
      this.Kq2_3 = null;
      this.rho_p0_1 = null;
      this.rho_p1_1 = null;
      this.rho_p0_2 = null;
      this.rho_p1_2 = null;
      this.rho_p0_3 = null;
      this.rho_p1_3 = null;
      this.rho_q0_1 = null;
      this.rho_q1_1 = null;
      this.rho_q2_1 = null;
      this.rho_q0_2 = null;
      this.rho_q1_2 = null;
      this.rho_q2_2 = null;
      this.rho_q0_3 = null;
      this.rho_Q1_3 = null;
      this.rho_q2_3 = null;
      this.zeta_p = null;
      this.zeta_q = null;
      this.delTau_p = null;
      this.delTau_q = null;
      this.moments = null;
      this.thrust = null;
      this.hatM_1 = null;
      this.hatM_2 = null;
      this.hatM_3 = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('sp')) {
        this.sp = initObj.sp
      }
      else {
        this.sp = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('sq')) {
        this.sq = initObj.sq
      }
      else {
        this.sq = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angular_acceleration')) {
        this.angular_acceleration = initObj.angular_acceleration
      }
      else {
        this.angular_acceleration = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('position_error')) {
        this.position_error = initObj.position_error
      }
      else {
        this.position_error = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('velocity_error')) {
        this.velocity_error = initObj.velocity_error
      }
      else {
        this.velocity_error = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('position_error_integral')) {
        this.position_error_integral = initObj.position_error_integral
      }
      else {
        this.position_error_integral = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angle_error')) {
        this.angle_error = initObj.angle_error
      }
      else {
        this.angle_error = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angle_rate_error')) {
        this.angle_rate_error = initObj.angle_rate_error
      }
      else {
        this.angle_rate_error = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kp0_1')) {
        this.Kp0_1 = initObj.Kp0_1
      }
      else {
        this.Kp0_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kp1_1')) {
        this.Kp1_1 = initObj.Kp1_1
      }
      else {
        this.Kp1_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kp0_2')) {
        this.Kp0_2 = initObj.Kp0_2
      }
      else {
        this.Kp0_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kp1_2')) {
        this.Kp1_2 = initObj.Kp1_2
      }
      else {
        this.Kp1_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kp0_3')) {
        this.Kp0_3 = initObj.Kp0_3
      }
      else {
        this.Kp0_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kp1_3')) {
        this.Kp1_3 = initObj.Kp1_3
      }
      else {
        this.Kp1_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq0_1')) {
        this.Kq0_1 = initObj.Kq0_1
      }
      else {
        this.Kq0_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq1_1')) {
        this.Kq1_1 = initObj.Kq1_1
      }
      else {
        this.Kq1_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq2_1')) {
        this.Kq2_1 = initObj.Kq2_1
      }
      else {
        this.Kq2_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq0_2')) {
        this.Kq0_2 = initObj.Kq0_2
      }
      else {
        this.Kq0_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq1_2')) {
        this.Kq1_2 = initObj.Kq1_2
      }
      else {
        this.Kq1_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq2_2')) {
        this.Kq2_2 = initObj.Kq2_2
      }
      else {
        this.Kq2_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq0_3')) {
        this.Kq0_3 = initObj.Kq0_3
      }
      else {
        this.Kq0_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq1_3')) {
        this.Kq1_3 = initObj.Kq1_3
      }
      else {
        this.Kq1_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('Kq2_3')) {
        this.Kq2_3 = initObj.Kq2_3
      }
      else {
        this.Kq2_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_p0_1')) {
        this.rho_p0_1 = initObj.rho_p0_1
      }
      else {
        this.rho_p0_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_p1_1')) {
        this.rho_p1_1 = initObj.rho_p1_1
      }
      else {
        this.rho_p1_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_p0_2')) {
        this.rho_p0_2 = initObj.rho_p0_2
      }
      else {
        this.rho_p0_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_p1_2')) {
        this.rho_p1_2 = initObj.rho_p1_2
      }
      else {
        this.rho_p1_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_p0_3')) {
        this.rho_p0_3 = initObj.rho_p0_3
      }
      else {
        this.rho_p0_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_p1_3')) {
        this.rho_p1_3 = initObj.rho_p1_3
      }
      else {
        this.rho_p1_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_q0_1')) {
        this.rho_q0_1 = initObj.rho_q0_1
      }
      else {
        this.rho_q0_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_q1_1')) {
        this.rho_q1_1 = initObj.rho_q1_1
      }
      else {
        this.rho_q1_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_q2_1')) {
        this.rho_q2_1 = initObj.rho_q2_1
      }
      else {
        this.rho_q2_1 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_q0_2')) {
        this.rho_q0_2 = initObj.rho_q0_2
      }
      else {
        this.rho_q0_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_q1_2')) {
        this.rho_q1_2 = initObj.rho_q1_2
      }
      else {
        this.rho_q1_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_q2_2')) {
        this.rho_q2_2 = initObj.rho_q2_2
      }
      else {
        this.rho_q2_2 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_q0_3')) {
        this.rho_q0_3 = initObj.rho_q0_3
      }
      else {
        this.rho_q0_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_Q1_3')) {
        this.rho_Q1_3 = initObj.rho_Q1_3
      }
      else {
        this.rho_Q1_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rho_q2_3')) {
        this.rho_q2_3 = initObj.rho_q2_3
      }
      else {
        this.rho_q2_3 = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('zeta_p')) {
        this.zeta_p = initObj.zeta_p
      }
      else {
        this.zeta_p = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('zeta_q')) {
        this.zeta_q = initObj.zeta_q
      }
      else {
        this.zeta_q = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('delTau_p')) {
        this.delTau_p = initObj.delTau_p
      }
      else {
        this.delTau_p = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('delTau_q')) {
        this.delTau_q = initObj.delTau_q
      }
      else {
        this.delTau_q = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('moments')) {
        this.moments = initObj.moments
      }
      else {
        this.moments = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0.0;
      }
      if (initObj.hasOwnProperty('hatM_1')) {
        this.hatM_1 = initObj.hatM_1
      }
      else {
        this.hatM_1 = 0.0;
      }
      if (initObj.hasOwnProperty('hatM_2')) {
        this.hatM_2 = initObj.hatM_2
      }
      else {
        this.hatM_2 = 0.0;
      }
      if (initObj.hasOwnProperty('hatM_3')) {
        this.hatM_3 = initObj.hatM_3
      }
      else {
        this.hatM_3 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SwDataMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [sp]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.sp, buffer, bufferOffset);
    // Serialize message field [sq]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.sq, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.acceleration, buffer, bufferOffset);
    // Serialize message field [angular_acceleration]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular_acceleration, buffer, bufferOffset);
    // Serialize message field [position_error]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position_error, buffer, bufferOffset);
    // Serialize message field [velocity_error]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.velocity_error, buffer, bufferOffset);
    // Serialize message field [position_error_integral]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.position_error_integral, buffer, bufferOffset);
    // Serialize message field [angle_error]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angle_error, buffer, bufferOffset);
    // Serialize message field [angle_rate_error]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angle_rate_error, buffer, bufferOffset);
    // Serialize message field [Kp0_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kp0_1, buffer, bufferOffset);
    // Serialize message field [Kp1_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kp1_1, buffer, bufferOffset);
    // Serialize message field [Kp0_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kp0_2, buffer, bufferOffset);
    // Serialize message field [Kp1_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kp1_2, buffer, bufferOffset);
    // Serialize message field [Kp0_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kp0_3, buffer, bufferOffset);
    // Serialize message field [Kp1_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kp1_3, buffer, bufferOffset);
    // Serialize message field [Kq0_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq0_1, buffer, bufferOffset);
    // Serialize message field [Kq1_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq1_1, buffer, bufferOffset);
    // Serialize message field [Kq2_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq2_1, buffer, bufferOffset);
    // Serialize message field [Kq0_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq0_2, buffer, bufferOffset);
    // Serialize message field [Kq1_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq1_2, buffer, bufferOffset);
    // Serialize message field [Kq2_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq2_2, buffer, bufferOffset);
    // Serialize message field [Kq0_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq0_3, buffer, bufferOffset);
    // Serialize message field [Kq1_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq1_3, buffer, bufferOffset);
    // Serialize message field [Kq2_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.Kq2_3, buffer, bufferOffset);
    // Serialize message field [rho_p0_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_p0_1, buffer, bufferOffset);
    // Serialize message field [rho_p1_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_p1_1, buffer, bufferOffset);
    // Serialize message field [rho_p0_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_p0_2, buffer, bufferOffset);
    // Serialize message field [rho_p1_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_p1_2, buffer, bufferOffset);
    // Serialize message field [rho_p0_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_p0_3, buffer, bufferOffset);
    // Serialize message field [rho_p1_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_p1_3, buffer, bufferOffset);
    // Serialize message field [rho_q0_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_q0_1, buffer, bufferOffset);
    // Serialize message field [rho_q1_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_q1_1, buffer, bufferOffset);
    // Serialize message field [rho_q2_1]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_q2_1, buffer, bufferOffset);
    // Serialize message field [rho_q0_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_q0_2, buffer, bufferOffset);
    // Serialize message field [rho_q1_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_q1_2, buffer, bufferOffset);
    // Serialize message field [rho_q2_2]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_q2_2, buffer, bufferOffset);
    // Serialize message field [rho_q0_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_q0_3, buffer, bufferOffset);
    // Serialize message field [rho_Q1_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_Q1_3, buffer, bufferOffset);
    // Serialize message field [rho_q2_3]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rho_q2_3, buffer, bufferOffset);
    // Serialize message field [zeta_p]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.zeta_p, buffer, bufferOffset);
    // Serialize message field [zeta_q]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.zeta_q, buffer, bufferOffset);
    // Serialize message field [delTau_p]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.delTau_p, buffer, bufferOffset);
    // Serialize message field [delTau_q]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.delTau_q, buffer, bufferOffset);
    // Serialize message field [moments]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.moments, buffer, bufferOffset);
    // Serialize message field [thrust]
    bufferOffset = _serializer.float64(obj.thrust, buffer, bufferOffset);
    // Serialize message field [hatM_1]
    bufferOffset = _serializer.float64(obj.hatM_1, buffer, bufferOffset);
    // Serialize message field [hatM_2]
    bufferOffset = _serializer.float64(obj.hatM_2, buffer, bufferOffset);
    // Serialize message field [hatM_3]
    bufferOffset = _serializer.float64(obj.hatM_3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SwDataMsg
    let len;
    let data = new SwDataMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [sp]
    data.sp = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [sq]
    data.sq = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_acceleration]
    data.angular_acceleration = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_error]
    data.position_error = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity_error]
    data.velocity_error = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [position_error_integral]
    data.position_error_integral = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angle_error]
    data.angle_error = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angle_rate_error]
    data.angle_rate_error = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kp0_1]
    data.Kp0_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kp1_1]
    data.Kp1_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kp0_2]
    data.Kp0_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kp1_2]
    data.Kp1_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kp0_3]
    data.Kp0_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kp1_3]
    data.Kp1_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq0_1]
    data.Kq0_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq1_1]
    data.Kq1_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq2_1]
    data.Kq2_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq0_2]
    data.Kq0_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq1_2]
    data.Kq1_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq2_2]
    data.Kq2_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq0_3]
    data.Kq0_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq1_3]
    data.Kq1_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [Kq2_3]
    data.Kq2_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_p0_1]
    data.rho_p0_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_p1_1]
    data.rho_p1_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_p0_2]
    data.rho_p0_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_p1_2]
    data.rho_p1_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_p0_3]
    data.rho_p0_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_p1_3]
    data.rho_p1_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_q0_1]
    data.rho_q0_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_q1_1]
    data.rho_q1_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_q2_1]
    data.rho_q2_1 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_q0_2]
    data.rho_q0_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_q1_2]
    data.rho_q1_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_q2_2]
    data.rho_q2_2 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_q0_3]
    data.rho_q0_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_Q1_3]
    data.rho_Q1_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rho_q2_3]
    data.rho_q2_3 = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [zeta_p]
    data.zeta_p = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [zeta_q]
    data.zeta_q = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [delTau_p]
    data.delTau_p = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [delTau_q]
    data.delTau_q = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [moments]
    data.moments = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [hatM_1]
    data.hatM_1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [hatM_2]
    data.hatM_2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [hatM_3]
    data.hatM_3 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 1088;
  }

  static datatype() {
    // Returns string type for a message object
    return 'msg_check/SwDataMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da1fe80469bc6c23b275683797085357';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    geometry_msgs/Vector3 sp
    geometry_msgs/Vector3 sq
    geometry_msgs/Vector3 acceleration
    geometry_msgs/Vector3 angular_acceleration
    geometry_msgs/Vector3 position_error
    geometry_msgs/Vector3 velocity_error
    geometry_msgs/Vector3 position_error_integral
    geometry_msgs/Vector3 angle_error
    geometry_msgs/Vector3 angle_rate_error
    geometry_msgs/Vector3 Kp0_1
    geometry_msgs/Vector3 Kp1_1
    geometry_msgs/Vector3 Kp0_2
    geometry_msgs/Vector3 Kp1_2
    geometry_msgs/Vector3 Kp0_3
    geometry_msgs/Vector3 Kp1_3
    geometry_msgs/Vector3 Kq0_1
    geometry_msgs/Vector3 Kq1_1
    geometry_msgs/Vector3 Kq2_1
    geometry_msgs/Vector3 Kq0_2
    geometry_msgs/Vector3 Kq1_2
    geometry_msgs/Vector3 Kq2_2
    geometry_msgs/Vector3 Kq0_3
    geometry_msgs/Vector3 Kq1_3
    geometry_msgs/Vector3 Kq2_3
    geometry_msgs/Vector3 rho_p0_1
    geometry_msgs/Vector3 rho_p1_1
    geometry_msgs/Vector3 rho_p0_2
    geometry_msgs/Vector3 rho_p1_2
    geometry_msgs/Vector3 rho_p0_3
    geometry_msgs/Vector3 rho_p1_3
    geometry_msgs/Vector3 rho_q0_1
    geometry_msgs/Vector3 rho_q1_1
    geometry_msgs/Vector3 rho_q2_1
    geometry_msgs/Vector3 rho_q0_2
    geometry_msgs/Vector3 rho_q1_2
    geometry_msgs/Vector3 rho_q2_2
    geometry_msgs/Vector3 rho_q0_3
    geometry_msgs/Vector3 rho_Q1_3
    geometry_msgs/Vector3 rho_q2_3
    geometry_msgs/Vector3 zeta_p
    geometry_msgs/Vector3 zeta_q
    geometry_msgs/Vector3 delTau_p
    geometry_msgs/Vector3 delTau_q
    geometry_msgs/Vector3 moments
    float64 thrust
    float64 hatM_1
    float64 hatM_2
    float64 hatM_3
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SwDataMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.sp !== undefined) {
      resolved.sp = geometry_msgs.msg.Vector3.Resolve(msg.sp)
    }
    else {
      resolved.sp = new geometry_msgs.msg.Vector3()
    }

    if (msg.sq !== undefined) {
      resolved.sq = geometry_msgs.msg.Vector3.Resolve(msg.sq)
    }
    else {
      resolved.sq = new geometry_msgs.msg.Vector3()
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = geometry_msgs.msg.Vector3.Resolve(msg.acceleration)
    }
    else {
      resolved.acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.angular_acceleration !== undefined) {
      resolved.angular_acceleration = geometry_msgs.msg.Vector3.Resolve(msg.angular_acceleration)
    }
    else {
      resolved.angular_acceleration = new geometry_msgs.msg.Vector3()
    }

    if (msg.position_error !== undefined) {
      resolved.position_error = geometry_msgs.msg.Vector3.Resolve(msg.position_error)
    }
    else {
      resolved.position_error = new geometry_msgs.msg.Vector3()
    }

    if (msg.velocity_error !== undefined) {
      resolved.velocity_error = geometry_msgs.msg.Vector3.Resolve(msg.velocity_error)
    }
    else {
      resolved.velocity_error = new geometry_msgs.msg.Vector3()
    }

    if (msg.position_error_integral !== undefined) {
      resolved.position_error_integral = geometry_msgs.msg.Vector3.Resolve(msg.position_error_integral)
    }
    else {
      resolved.position_error_integral = new geometry_msgs.msg.Vector3()
    }

    if (msg.angle_error !== undefined) {
      resolved.angle_error = geometry_msgs.msg.Vector3.Resolve(msg.angle_error)
    }
    else {
      resolved.angle_error = new geometry_msgs.msg.Vector3()
    }

    if (msg.angle_rate_error !== undefined) {
      resolved.angle_rate_error = geometry_msgs.msg.Vector3.Resolve(msg.angle_rate_error)
    }
    else {
      resolved.angle_rate_error = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kp0_1 !== undefined) {
      resolved.Kp0_1 = geometry_msgs.msg.Vector3.Resolve(msg.Kp0_1)
    }
    else {
      resolved.Kp0_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kp1_1 !== undefined) {
      resolved.Kp1_1 = geometry_msgs.msg.Vector3.Resolve(msg.Kp1_1)
    }
    else {
      resolved.Kp1_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kp0_2 !== undefined) {
      resolved.Kp0_2 = geometry_msgs.msg.Vector3.Resolve(msg.Kp0_2)
    }
    else {
      resolved.Kp0_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kp1_2 !== undefined) {
      resolved.Kp1_2 = geometry_msgs.msg.Vector3.Resolve(msg.Kp1_2)
    }
    else {
      resolved.Kp1_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kp0_3 !== undefined) {
      resolved.Kp0_3 = geometry_msgs.msg.Vector3.Resolve(msg.Kp0_3)
    }
    else {
      resolved.Kp0_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kp1_3 !== undefined) {
      resolved.Kp1_3 = geometry_msgs.msg.Vector3.Resolve(msg.Kp1_3)
    }
    else {
      resolved.Kp1_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq0_1 !== undefined) {
      resolved.Kq0_1 = geometry_msgs.msg.Vector3.Resolve(msg.Kq0_1)
    }
    else {
      resolved.Kq0_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq1_1 !== undefined) {
      resolved.Kq1_1 = geometry_msgs.msg.Vector3.Resolve(msg.Kq1_1)
    }
    else {
      resolved.Kq1_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq2_1 !== undefined) {
      resolved.Kq2_1 = geometry_msgs.msg.Vector3.Resolve(msg.Kq2_1)
    }
    else {
      resolved.Kq2_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq0_2 !== undefined) {
      resolved.Kq0_2 = geometry_msgs.msg.Vector3.Resolve(msg.Kq0_2)
    }
    else {
      resolved.Kq0_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq1_2 !== undefined) {
      resolved.Kq1_2 = geometry_msgs.msg.Vector3.Resolve(msg.Kq1_2)
    }
    else {
      resolved.Kq1_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq2_2 !== undefined) {
      resolved.Kq2_2 = geometry_msgs.msg.Vector3.Resolve(msg.Kq2_2)
    }
    else {
      resolved.Kq2_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq0_3 !== undefined) {
      resolved.Kq0_3 = geometry_msgs.msg.Vector3.Resolve(msg.Kq0_3)
    }
    else {
      resolved.Kq0_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq1_3 !== undefined) {
      resolved.Kq1_3 = geometry_msgs.msg.Vector3.Resolve(msg.Kq1_3)
    }
    else {
      resolved.Kq1_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.Kq2_3 !== undefined) {
      resolved.Kq2_3 = geometry_msgs.msg.Vector3.Resolve(msg.Kq2_3)
    }
    else {
      resolved.Kq2_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_p0_1 !== undefined) {
      resolved.rho_p0_1 = geometry_msgs.msg.Vector3.Resolve(msg.rho_p0_1)
    }
    else {
      resolved.rho_p0_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_p1_1 !== undefined) {
      resolved.rho_p1_1 = geometry_msgs.msg.Vector3.Resolve(msg.rho_p1_1)
    }
    else {
      resolved.rho_p1_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_p0_2 !== undefined) {
      resolved.rho_p0_2 = geometry_msgs.msg.Vector3.Resolve(msg.rho_p0_2)
    }
    else {
      resolved.rho_p0_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_p1_2 !== undefined) {
      resolved.rho_p1_2 = geometry_msgs.msg.Vector3.Resolve(msg.rho_p1_2)
    }
    else {
      resolved.rho_p1_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_p0_3 !== undefined) {
      resolved.rho_p0_3 = geometry_msgs.msg.Vector3.Resolve(msg.rho_p0_3)
    }
    else {
      resolved.rho_p0_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_p1_3 !== undefined) {
      resolved.rho_p1_3 = geometry_msgs.msg.Vector3.Resolve(msg.rho_p1_3)
    }
    else {
      resolved.rho_p1_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_q0_1 !== undefined) {
      resolved.rho_q0_1 = geometry_msgs.msg.Vector3.Resolve(msg.rho_q0_1)
    }
    else {
      resolved.rho_q0_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_q1_1 !== undefined) {
      resolved.rho_q1_1 = geometry_msgs.msg.Vector3.Resolve(msg.rho_q1_1)
    }
    else {
      resolved.rho_q1_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_q2_1 !== undefined) {
      resolved.rho_q2_1 = geometry_msgs.msg.Vector3.Resolve(msg.rho_q2_1)
    }
    else {
      resolved.rho_q2_1 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_q0_2 !== undefined) {
      resolved.rho_q0_2 = geometry_msgs.msg.Vector3.Resolve(msg.rho_q0_2)
    }
    else {
      resolved.rho_q0_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_q1_2 !== undefined) {
      resolved.rho_q1_2 = geometry_msgs.msg.Vector3.Resolve(msg.rho_q1_2)
    }
    else {
      resolved.rho_q1_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_q2_2 !== undefined) {
      resolved.rho_q2_2 = geometry_msgs.msg.Vector3.Resolve(msg.rho_q2_2)
    }
    else {
      resolved.rho_q2_2 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_q0_3 !== undefined) {
      resolved.rho_q0_3 = geometry_msgs.msg.Vector3.Resolve(msg.rho_q0_3)
    }
    else {
      resolved.rho_q0_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_Q1_3 !== undefined) {
      resolved.rho_Q1_3 = geometry_msgs.msg.Vector3.Resolve(msg.rho_Q1_3)
    }
    else {
      resolved.rho_Q1_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.rho_q2_3 !== undefined) {
      resolved.rho_q2_3 = geometry_msgs.msg.Vector3.Resolve(msg.rho_q2_3)
    }
    else {
      resolved.rho_q2_3 = new geometry_msgs.msg.Vector3()
    }

    if (msg.zeta_p !== undefined) {
      resolved.zeta_p = geometry_msgs.msg.Vector3.Resolve(msg.zeta_p)
    }
    else {
      resolved.zeta_p = new geometry_msgs.msg.Vector3()
    }

    if (msg.zeta_q !== undefined) {
      resolved.zeta_q = geometry_msgs.msg.Vector3.Resolve(msg.zeta_q)
    }
    else {
      resolved.zeta_q = new geometry_msgs.msg.Vector3()
    }

    if (msg.delTau_p !== undefined) {
      resolved.delTau_p = geometry_msgs.msg.Vector3.Resolve(msg.delTau_p)
    }
    else {
      resolved.delTau_p = new geometry_msgs.msg.Vector3()
    }

    if (msg.delTau_q !== undefined) {
      resolved.delTau_q = geometry_msgs.msg.Vector3.Resolve(msg.delTau_q)
    }
    else {
      resolved.delTau_q = new geometry_msgs.msg.Vector3()
    }

    if (msg.moments !== undefined) {
      resolved.moments = geometry_msgs.msg.Vector3.Resolve(msg.moments)
    }
    else {
      resolved.moments = new geometry_msgs.msg.Vector3()
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0.0
    }

    if (msg.hatM_1 !== undefined) {
      resolved.hatM_1 = msg.hatM_1;
    }
    else {
      resolved.hatM_1 = 0.0
    }

    if (msg.hatM_2 !== undefined) {
      resolved.hatM_2 = msg.hatM_2;
    }
    else {
      resolved.hatM_2 = 0.0
    }

    if (msg.hatM_3 !== undefined) {
      resolved.hatM_3 = msg.hatM_3;
    }
    else {
      resolved.hatM_3 = 0.0
    }

    return resolved;
    }
};

module.exports = SwDataMsg;
