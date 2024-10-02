/**
 * @file SetupPoses.h
 *
 * Declaration of a representation that contains information about
 * the pose from which the robots enter the pitch when the game state
 * switches from INITIAL to READY.
 *
 * @author Tim Laue
 */

#pragma once

#include "Platform/BHAssert.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include <vector>

/**
 * @struct SetupPoses
 * A representation that contains a list of poses from which the robots enter
 * the pitch when the game state switches from INITIAL to READY.
 */
STREAMABLE(SetupPoses,
{
  /** The pose of a robot before entering the field */
  STREAMABLE(SetupPose,
  {,
    (int) playerNumber,           /*< The player number of the robot */
    (Vector2f) position,          /*< The position (in global field coordinates) at which the robot is placed */
    (Vector2f) turnedTowards,     /*< The position (in global field coordinates) at which the robot is turned (looking at) */
  });

  /** Convenience function to find the correct pose given the player number.
   *  The list of poses is not ordered by numbers.
   *  It has to be made sure that the config file contains the entry. Otherwise -> ASSERT!
   *  Exception (for demos and tests): If the list has only one entry, this entry is returned, no matter which number the robot has.
   *  @param number The player number (starting with 1, as the real number)
   *  @return A reference to the pose for setup
   */
  const SetupPose &getPoseOfRobot(int number, const GameInfo& gameInfo, const OwnTeamInfo& ownTeamInfo) const,
  (std::vector<SetupPose>)posesAttack,  /*< A list of all available robot poses, not ordered by number, in Attack mode */
  (std::vector<SetupPose>)posesDefence, /*< A list of all available robot poses, not ordered by number, in Defence mode */
});

inline const SetupPoses::SetupPose& SetupPoses::getPoseOfRobot(int number, const GameInfo& gameInfo, const OwnTeamInfo& ownTeamInfo) const
{
  if (gameInfo.kickingTeam == ownTeamInfo.teamNumber)
  {
    ASSERT(posesAttack.size() > 0);
    if (posesAttack.size() == 1)
      return posesAttack[0];
    for (const auto &pose : posesAttack)
      if (pose.playerNumber == number)
        return pose;
    ASSERT(false);
    return posesAttack[0]; // Dummy line to avoid compiler complaints
  }
  else
  {
    ASSERT(posesDefence.size() > 0);
    if (posesDefence.size() == 1)
      return posesDefence[0];
    for (const auto &pose : posesDefence)
      if (pose.playerNumber == number)
        return pose;
    ASSERT(false);
    return posesDefence[0]; // Dummy line to avoid compiler complaints
  }
}
