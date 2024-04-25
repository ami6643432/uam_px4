
"use strict";

let Simple = require('./Simple.js');
let MigratedExplicit = require('./MigratedExplicit.js');
let MigratedMixed = require('./MigratedMixed.js');
let SimpleMigrated = require('./SimpleMigrated.js');
let Converged = require('./Converged.js');
let MigratedImplicit = require('./MigratedImplicit.js');
let MigratedAddSub = require('./MigratedAddSub.js');
let Unmigrated = require('./Unmigrated.js');
let SubUnmigrated = require('./SubUnmigrated.js');
let Constants = require('./Constants.js');
let PartiallyMigrated = require('./PartiallyMigrated.js');
let Renamed5 = require('./Renamed5.js');

module.exports = {
  Simple: Simple,
  MigratedExplicit: MigratedExplicit,
  MigratedMixed: MigratedMixed,
  SimpleMigrated: SimpleMigrated,
  Converged: Converged,
  MigratedImplicit: MigratedImplicit,
  MigratedAddSub: MigratedAddSub,
  Unmigrated: Unmigrated,
  SubUnmigrated: SubUnmigrated,
  Constants: Constants,
  PartiallyMigrated: PartiallyMigrated,
  Renamed5: Renamed5,
};
