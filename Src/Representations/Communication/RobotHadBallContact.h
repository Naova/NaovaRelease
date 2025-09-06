/**
 * @file RobotHadBallContact.h
 *
 */

#pragma once

#include "Tools/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"
#include "Tools/Streams/AutoStreamable.h"


STREAMABLE(RobotHadBallContact, COMMA BHumanCompressedMessageParticle<RobotHadBallContact>
{,
  (bool)(false) hadContact, /**< Whether the robot has touched the ball since the last kickoff. */
  (uint32_t)(0) timeLastKickoff, /**< The timestamp of the last kickoff. */
});