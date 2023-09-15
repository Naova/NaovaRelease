/**
 * @file LibCodeRelease.h
 */

#pragma once
#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Role.h"

STREAMABLE(LibCodeRelease,
{
FUNCTION(bool(float value, float min, float max)) between;
FUNCTION(float(float value, float min, float max)) clamp,

(float)(0.f) angleToOppGoal,
(float)(0.f) angleToOwnGoal,
(Vector2f)(Vector2f::Zero()) desiredKickPos,
(Vector2f)(Vector2f::Zero()) desiredPos,
(Vector2f)(Vector2f::Zero()) defenderDesiredPos,
(int)(0) timeSinceBallWasSeen,
(int)(0) nbOfDefender,
(int)(0) nbOfRightDefender,
(int)(0) nbOfLeftDefender,
(int)(0) nbOfKeeper,
(int)(0) nbOfStriker,
(int)(0) nbOfSupporter,
(int)(0) nbOfPlayers,
(int)(3000) waitToChangeRole,
(bool)(false) closerToTheBall,
(bool)(false) closerToTheBallDef,
(bool)(false) defenderCloserToTheBall,
(bool)(false) ballIsInGoalArea,
(bool)(false) robotIsInPenaltyArea,
 (float) (120.f) footLength ,
 (float) (150.f) safeDistance ,
 (float) (1000.f) defenderDistanceForKick,
 (float) (400.f) strikerDistanceForKick,
 (float) (400.f) supporterDistanceForKick,
 (float) (1000.f) keeperDistanceForKick,
 (float) (0.f) supporterDesiredAngle,
 (float) (3800.f) kickFarRange,
 (Vector2f) (Vector2f::Zero()) supporterDesiredKickPos,
 ((Role) RoleType)(undefined) desiredRole,
 (int) (0) lastGameState,
 (bool) (false) ballModelIsValid,
});