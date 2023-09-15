/**
 * @file Representations/Configuration/BehaviorParameters/BehaviorParameters.h
 *
 * The file declares a struct that contains frequently used parameters of the behavior which can be modified.
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"
#include "Tools/Math/Angle.h"

STREAMABLE(BehaviorParameters,
{,
  (bool)(false) enableWalkStraight,
  (int)(7000) ballNotSeenTimeOut,
  (float)(850.f) safeDistance,
  //ArmInBack
  (int)(500) maxObstacleDistanceArmsInBack,
  (float)(0.25f) leftAngleArmsInBack,
});
