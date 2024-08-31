/**
 * @file KickOff.h
 *
 * This file declares the representation of a kick-off.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "SetPlay.h"
#include "Tactic.h"
#include "Tools/Math/Pose2f.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Streams/Enum.h"
#include <vector>

STREAMABLE_WITH_BASE(KickOff, SetPlay,
{,
});

STREAMABLE_WITH_BASE(OwnKickOff, KickOff,
{
  ENUM(Type,
  {,
    offensiveKickOff,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(ownKickOffBegin + static_cast<unsigned>(type));
  },
});

STREAMABLE_WITH_BASE(OpponentKickOff, KickOff,
{
  ENUM(Type,
  {,
    defensiveKickOff,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(opponentKickOffBegin + static_cast<unsigned>(type));
  },
});
