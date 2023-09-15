/**
 * @file Representations/BehaviorControl/TimeToReachBall.h
 *
 * Representation of the estimated time to reach the ball
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Representations/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"
#include <limits>
#include <cmath>

/**
 * @struct TimeToReachBall
 * Representation of the Time to reach the ball
 */
STREAMABLE(TimeToReachBall, COMMA public NaovaMessageParticule<undefined>
{
  /** NaovaMessageParticle functions */
  void operator >> (NaovaMessage& m) const override;
  void operator << (const NaovaMessage& m) override,

  (unsigned)(std::numeric_limits<unsigned>::max()) timeWhenReachBall,         /**< The estimated time when reach the ball */
  (unsigned)(std::numeric_limits<unsigned>::max()) timeWhenReachBallStriker,
});

inline void TimeToReachBall::operator >> (NaovaMessage& m) const
{
  // m.theNaovaStandardMessage.timeWhenReachBall = timeWhenReachBall;
  // m.theNaovaStandardMessage.timeWhenReachBallStriker = timeWhenReachBallStriker;
}

inline void TimeToReachBall::operator << (const NaovaMessage& m)
{
  // timeWhenReachBall = m.toLocalTimestamp(m.theNaovaStandardMessage.timeWhenReachBall);
  // timeWhenReachBallStriker = m.toLocalTimestamp(m.theNaovaStandardMessage.timeWhenReachBallStriker);
}
