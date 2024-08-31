/**
 * @file Representations/BehaviorControl/CurrentTactic.h
 *
 * Representation of the curent tactic
 *
 * @author Olivier St-Pierre
 * @author Jérémy Thu-Thon
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/Tactic.h"
#include "Tools/BehaviorControl/Strategy/PenaltyKick.h"
#include "Tools/BehaviorControl/Strategy/FreeKick.h"
#include "Tools/Streams/AutoStreamable.h"
#include <limits>

/**
 * @struct CurrentTactic
 * Representation of the position of the supporter role
 */
STREAMABLE(CurrentTactic,
{,
    (Tactic) currentTactic, /* The current tactic to apply */
    (Tactic::Type) currentTacticType, /* The current tactic type to apply */
    (SetPlay) currentSetPlay, /**< The current set play to apply */
    (FreeKick) currentFreeKick,
    (SetPlay::Type)(SetPlay::none) currentSetPlayType, /**< The current set play type to apply */
});
