
"use strict";

let GetJointPosition = require('./GetJointPosition.js')
let GetKinematicsPose = require('./GetKinematicsPose.js')
let SetActuatorState = require('./SetActuatorState.js')
let SetKinematicsPose = require('./SetKinematicsPose.js')
let SetDrawingTrajectory = require('./SetDrawingTrajectory.js')
let SetJointPosition = require('./SetJointPosition.js')

module.exports = {
  GetJointPosition: GetJointPosition,
  GetKinematicsPose: GetKinematicsPose,
  SetActuatorState: SetActuatorState,
  SetKinematicsPose: SetKinematicsPose,
  SetDrawingTrajectory: SetDrawingTrajectory,
  SetJointPosition: SetJointPosition,
};
