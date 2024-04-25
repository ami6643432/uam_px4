
"use strict";

let ParamGet = require('./ParamGet.js')
let FileWrite = require('./FileWrite.js')
let CommandVtolTransition = require('./CommandVtolTransition.js')
let LogRequestList = require('./LogRequestList.js')
let CommandTOL = require('./CommandTOL.js')
let FileClose = require('./FileClose.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let WaypointPull = require('./WaypointPull.js')
let CommandTriggerInterval = require('./CommandTriggerInterval.js')
let CommandLong = require('./CommandLong.js')
let LogRequestEnd = require('./LogRequestEnd.js')
let VehicleInfoGet = require('./VehicleInfoGet.js')
let ParamPull = require('./ParamPull.js')
let MountConfigure = require('./MountConfigure.js')
let CommandHome = require('./CommandHome.js')
let FileChecksum = require('./FileChecksum.js')
let SetMavFrame = require('./SetMavFrame.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileOpen = require('./FileOpen.js')
let WaypointPush = require('./WaypointPush.js')
let MessageInterval = require('./MessageInterval.js')
let FileRename = require('./FileRename.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let FileTruncate = require('./FileTruncate.js')
let FileList = require('./FileList.js')
let CommandInt = require('./CommandInt.js')
let ParamPush = require('./ParamPush.js')
let FileMakeDir = require('./FileMakeDir.js')
let CommandBool = require('./CommandBool.js')
let WaypointClear = require('./WaypointClear.js')
let CommandAck = require('./CommandAck.js')
let FileRemove = require('./FileRemove.js')
let FileRead = require('./FileRead.js')
let LogRequestData = require('./LogRequestData.js')
let SetMode = require('./SetMode.js')
let ParamSet = require('./ParamSet.js')
let StreamRate = require('./StreamRate.js')

module.exports = {
  ParamGet: ParamGet,
  FileWrite: FileWrite,
  CommandVtolTransition: CommandVtolTransition,
  LogRequestList: LogRequestList,
  CommandTOL: CommandTOL,
  FileClose: FileClose,
  CommandTriggerControl: CommandTriggerControl,
  WaypointPull: WaypointPull,
  CommandTriggerInterval: CommandTriggerInterval,
  CommandLong: CommandLong,
  LogRequestEnd: LogRequestEnd,
  VehicleInfoGet: VehicleInfoGet,
  ParamPull: ParamPull,
  MountConfigure: MountConfigure,
  CommandHome: CommandHome,
  FileChecksum: FileChecksum,
  SetMavFrame: SetMavFrame,
  WaypointSetCurrent: WaypointSetCurrent,
  FileOpen: FileOpen,
  WaypointPush: WaypointPush,
  MessageInterval: MessageInterval,
  FileRename: FileRename,
  FileRemoveDir: FileRemoveDir,
  FileTruncate: FileTruncate,
  FileList: FileList,
  CommandInt: CommandInt,
  ParamPush: ParamPush,
  FileMakeDir: FileMakeDir,
  CommandBool: CommandBool,
  WaypointClear: WaypointClear,
  CommandAck: CommandAck,
  FileRemove: FileRemove,
  FileRead: FileRead,
  LogRequestData: LogRequestData,
  SetMode: SetMode,
  ParamSet: ParamSet,
  StreamRate: StreamRate,
};
