/**
 * @file Representations/BehaviorControl/SupporterPositioning.h
 *
 * Representation of the position of the supporter role
 *
 * @author Olivier St-Pierre
 * @author Jérémy Thu-Thon
 */

#pragma once

#include "Tools/BehaviorControl/Strategy/Tactic.h"
#include "Tools/Streams/AutoStreamable.h"
#include <limits>

/**
 * @struct SupporterPositioning
 * Representation of the position of the supporter role
 */
STREAMABLE(SupporterPositioning,
{,
    (Tactic::Position::Type)(Tactic::Position::none) position, /**< The position of this agent in the tactic. */
    (Pose2f) basePose, /**< The pose from the tactic position. */
    (std::vector<Vector2f>) baseArea, /**< The region in the Voronoi diagram of the base poses in the tactic. */
});
