/**
 * @file LibCodeRelease.h
 */

#pragma once
#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

STREAMABLE(LibCodeRelease,
{
FUNCTION(bool(float value, float min, float max)) between;
FUNCTION(float(float value, float min, float max)) clamp;
FUNCTION(void(bool isMissing)) setMissingTeammate,

(float)(0.f) angleToOppGoal,
(float)(0.f) angleToOwnGoal,
(float)(0.f) angleToDesiredKick,
(Vector2f)(Vector2f::Zero()) desiredPos,
(int)(0) timeSinceBallWasSeen,
(int)(0) timeWhenRepportMisssing,
(int)(0) timeSinceMissingTeammate,
(int)(0) nbOfDefender,
(int)(0) nbOfKeeper,
(int)(0) nbOfStriker,
(int)(0) nbOfSupporter,
(int)(3000) waitToChangeRole,
(bool)(false) closerToTheBall,
(bool)(false) ballIsInPenaltyZone,
 (float) (120.f) footLength ,
 (float) (150.f) safeDistance ,
});