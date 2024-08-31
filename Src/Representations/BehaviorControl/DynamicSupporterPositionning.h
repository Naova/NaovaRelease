/**
 * @file Representations/BehaviorControl/DynamicSupporterPositionning
 *
 * Representation of the positionning strategy in offense
 *
 * @author Jérémy Thu-Thon
 */

#include "Tools/BehaviorControl/Strategy/Tactic.h"
#include "Tools/Streams/AutoStreamable.h"
#include <limits>

#pragma once

/**
 * @struct SupporterPositioning
 * Representation of the position of the supporter role
 */
STREAMABLE(DynamicSupporterPositionning,
{,
    (std::vector<Vector2f>) listePoints,
    (Pose2f) basePose,
});