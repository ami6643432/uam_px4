
"use strict";

let TestEmpty = require('./TestEmpty.js');
let TestArray = require('./TestArray.js');
let TestWithHeader = require('./TestWithHeader.js');
let TestStringInt = require('./TestStringInt.js');
let WithDuration = require('./WithDuration.js');
let ArrayOfVariableLength = require('./ArrayOfVariableLength.js');
let VariableLengthArrayOfExternal = require('./VariableLengthArrayOfExternal.js');
let FixedLengthStringArray = require('./FixedLengthStringArray.js');
let EmbeddedExternal = require('./EmbeddedExternal.js');
let HeaderNotFirstMember = require('./HeaderNotFirstMember.js');
let EmbeddedVariableLength = require('./EmbeddedVariableLength.js');
let VariableLength = require('./VariableLength.js');
let VariableLengthStringArray = require('./VariableLengthStringArray.js');
let WithMemberNamedHeaderThatIsNotAHeader = require('./WithMemberNamedHeaderThatIsNotAHeader.js');
let FixedLength = require('./FixedLength.js');
let CustomHeader = require('./CustomHeader.js');
let EmbeddedFixedLength = require('./EmbeddedFixedLength.js');
let WithHeader = require('./WithHeader.js');
let WithTime = require('./WithTime.js');
let Constants = require('./Constants.js');
let FixedLengthArrayOfExternal = require('./FixedLengthArrayOfExternal.js');
let ArrayOfFixedLength = require('./ArrayOfFixedLength.js');
let ThroughputMessage = require('./ThroughputMessage.js');
let LatencyMessage = require('./LatencyMessage.js');
let Point32 = require('./Point32.js');
let PointCloud = require('./PointCloud.js');
let ChannelFloat32 = require('./ChannelFloat32.js');

module.exports = {
  TestEmpty: TestEmpty,
  TestArray: TestArray,
  TestWithHeader: TestWithHeader,
  TestStringInt: TestStringInt,
  WithDuration: WithDuration,
  ArrayOfVariableLength: ArrayOfVariableLength,
  VariableLengthArrayOfExternal: VariableLengthArrayOfExternal,
  FixedLengthStringArray: FixedLengthStringArray,
  EmbeddedExternal: EmbeddedExternal,
  HeaderNotFirstMember: HeaderNotFirstMember,
  EmbeddedVariableLength: EmbeddedVariableLength,
  VariableLength: VariableLength,
  VariableLengthStringArray: VariableLengthStringArray,
  WithMemberNamedHeaderThatIsNotAHeader: WithMemberNamedHeaderThatIsNotAHeader,
  FixedLength: FixedLength,
  CustomHeader: CustomHeader,
  EmbeddedFixedLength: EmbeddedFixedLength,
  WithHeader: WithHeader,
  WithTime: WithTime,
  Constants: Constants,
  FixedLengthArrayOfExternal: FixedLengthArrayOfExternal,
  ArrayOfFixedLength: ArrayOfFixedLength,
  ThroughputMessage: ThroughputMessage,
  LatencyMessage: LatencyMessage,
  Point32: Point32,
  PointCloud: PointCloud,
  ChannelFloat32: ChannelFloat32,
};
