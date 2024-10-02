/**
 * @file PenaltyKick.h
 *
 * This file declares the representation of a penalty kick.
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

STREAMABLE_WITH_BASE(PenaltyKick, SetPlay,
{,
});

STREAMABLE_WITH_BASE(OwnPenaltyKick, PenaltyKick,
{
  ENUM(Type,
  {,
    theOneTrueOwnPenaltyKick,
    theOneTrueOwnPenaltyKick7v7,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(ownPenaltyKickBegin + static_cast<unsigned>(type));
  },
});

STREAMABLE_WITH_BASE(OpponentPenaltyKick, PenaltyKick,
{
  ENUM(Type,
  {,
    theOneTrueOpponentPenaltyKick,
    theOneTrueOpponentPenaltyKick7v7,
  });

  static SetPlay::Type toSetPlay(Type type)
  {
    return static_cast<SetPlay::Type>(opponentPenaltyKickBegin + static_cast<unsigned>(type));
  },
});
