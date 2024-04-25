
"use strict";

let GPSRTK = require('./GPSRTK.js');
let LogEntry = require('./LogEntry.js');
let ESCTelemetryItem = require('./ESCTelemetryItem.js');
let CommandCode = require('./CommandCode.js');
let ESCStatusItem = require('./ESCStatusItem.js');
let Vibration = require('./Vibration.js');
let Param = require('./Param.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let ESCInfo = require('./ESCInfo.js');
let Altitude = require('./Altitude.js');
let VehicleInfo = require('./VehicleInfo.js');
let ActuatorControl = require('./ActuatorControl.js');
let WaypointList = require('./WaypointList.js');
let FileEntry = require('./FileEntry.js');
let HilGPS = require('./HilGPS.js');
let MountControl = require('./MountControl.js');
let PositionTarget = require('./PositionTarget.js');
let ManualControl = require('./ManualControl.js');
let EstimatorStatus = require('./EstimatorStatus.js');
let VFR_HUD = require('./VFR_HUD.js');
let ESCInfoItem = require('./ESCInfoItem.js');
let ExtendedState = require('./ExtendedState.js');
let PlayTuneV2 = require('./PlayTuneV2.js');
let Waypoint = require('./Waypoint.js');
let TimesyncStatus = require('./TimesyncStatus.js');
let LogData = require('./LogData.js');
let CompanionProcessStatus = require('./CompanionProcessStatus.js');
let Trajectory = require('./Trajectory.js');
let Thrust = require('./Thrust.js');
let State = require('./State.js');
let RadioStatus = require('./RadioStatus.js');
let RCIn = require('./RCIn.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let OnboardComputerStatus = require('./OnboardComputerStatus.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let HilControls = require('./HilControls.js');
let HomePosition = require('./HomePosition.js');
let WheelOdomStamped = require('./WheelOdomStamped.js');
let RTKBaseline = require('./RTKBaseline.js');
let WaypointReached = require('./WaypointReached.js');
let NavControllerOutput = require('./NavControllerOutput.js');
let StatusText = require('./StatusText.js');
let RTCM = require('./RTCM.js');
let LandingTarget = require('./LandingTarget.js');
let ESCStatus = require('./ESCStatus.js');
let BatteryStatus = require('./BatteryStatus.js');
let ESCTelemetry = require('./ESCTelemetry.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let Tunnel = require('./Tunnel.js');
let DebugValue = require('./DebugValue.js');
let MagnetometerReporter = require('./MagnetometerReporter.js');
let CameraImageCaptured = require('./CameraImageCaptured.js');
let Mavlink = require('./Mavlink.js');
let ParamValue = require('./ParamValue.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let RCOut = require('./RCOut.js');
let GPSRAW = require('./GPSRAW.js');
let GPSINPUT = require('./GPSINPUT.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let HilSensor = require('./HilSensor.js');

module.exports = {
  GPSRTK: GPSRTK,
  LogEntry: LogEntry,
  ESCTelemetryItem: ESCTelemetryItem,
  CommandCode: CommandCode,
  ESCStatusItem: ESCStatusItem,
  Vibration: Vibration,
  Param: Param,
  OverrideRCIn: OverrideRCIn,
  ADSBVehicle: ADSBVehicle,
  CamIMUStamp: CamIMUStamp,
  ESCInfo: ESCInfo,
  Altitude: Altitude,
  VehicleInfo: VehicleInfo,
  ActuatorControl: ActuatorControl,
  WaypointList: WaypointList,
  FileEntry: FileEntry,
  HilGPS: HilGPS,
  MountControl: MountControl,
  PositionTarget: PositionTarget,
  ManualControl: ManualControl,
  EstimatorStatus: EstimatorStatus,
  VFR_HUD: VFR_HUD,
  ESCInfoItem: ESCInfoItem,
  ExtendedState: ExtendedState,
  PlayTuneV2: PlayTuneV2,
  Waypoint: Waypoint,
  TimesyncStatus: TimesyncStatus,
  LogData: LogData,
  CompanionProcessStatus: CompanionProcessStatus,
  Trajectory: Trajectory,
  Thrust: Thrust,
  State: State,
  RadioStatus: RadioStatus,
  RCIn: RCIn,
  AttitudeTarget: AttitudeTarget,
  OnboardComputerStatus: OnboardComputerStatus,
  HilStateQuaternion: HilStateQuaternion,
  HilControls: HilControls,
  HomePosition: HomePosition,
  WheelOdomStamped: WheelOdomStamped,
  RTKBaseline: RTKBaseline,
  WaypointReached: WaypointReached,
  NavControllerOutput: NavControllerOutput,
  StatusText: StatusText,
  RTCM: RTCM,
  LandingTarget: LandingTarget,
  ESCStatus: ESCStatus,
  BatteryStatus: BatteryStatus,
  ESCTelemetry: ESCTelemetry,
  OpticalFlowRad: OpticalFlowRad,
  Tunnel: Tunnel,
  DebugValue: DebugValue,
  MagnetometerReporter: MagnetometerReporter,
  CameraImageCaptured: CameraImageCaptured,
  Mavlink: Mavlink,
  ParamValue: ParamValue,
  HilActuatorControls: HilActuatorControls,
  RCOut: RCOut,
  GPSRAW: GPSRAW,
  GPSINPUT: GPSINPUT,
  GlobalPositionTarget: GlobalPositionTarget,
  HilSensor: HilSensor,
};
