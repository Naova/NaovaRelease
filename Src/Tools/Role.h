#pragma once

#include "Representations/BehaviorControl/RoleCheck.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"

/**
 * @struct Role
 * Representation of a robot's behavior role
 */

STREAMABLE(Role,
{
  /** The different roles */
  ENUM(RoleType, uint8_t
  {,
    undefined,
    keeper,
    striker,
    defender,
    leftDefender,
    rightDefender,
    supporter,
    none,
  });

  bool isGoalkeeper() const;

  CHECK_OSCILLATION(role, RoleType, undefined, "Role", 5000)
  /** Draws the current role next to the robot on the field view (in local robot coordinates) */
  void draw() const,

  /** Instance of role */
  (RoleType)(undefined) role,
  (RoleType)(undefined) lastRole,
});