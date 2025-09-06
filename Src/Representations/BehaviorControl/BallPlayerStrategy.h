/**
 * @file Representations/BehaviorControl/BallPlayerStrategy
 *
 * Representation of the strategy the ballplayer will use
 *
 * @author Elias
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include <limits>
#include "Representations/Modeling/ObstacleModel.h"

/**
 * @struct BallPlayerStrategy
 * Representation of the strategy the ballplayer will use
 */
STREAMABLE(BallPlayerStrategy,
{
    ENUM(Strategy,
    {,
        unknown,
        dribble,
        kickAtGoal,
        pass,
        duel,
        clear,
    }),

    (Strategy)(unknown) currentStrategy, /* The strategy used by the ballPlayer, this will determine which card will be played */
    (Obstacle) closestEnemy,
    (Vector2f) (Vector2f(0.f, 0.f)) targetForPass, /* Absolute vector from the ballPlayer to the teammate we want to pass to */
    (Vector2f) (Vector2f(0.f, 0.f)) targetForKick, /* Absolute vector from the ballPlayer to the target we want to goal kick to */
    (Vector2f) (Vector2f(0.f, 0.f)) targetForClear, /* Absolute vector from the ballPlayer to the target we want to clear to */
    (bool) (false) BPisBlockedInPenaltyArea,
});