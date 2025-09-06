/**
 * @file DribblePath.h
 *
 * Declaration of a representation that contains additional information
 * about the dribble path.
 *
 * @author Aissa
 */

#pragma once

#include "Tools/Math/Eigen.h"  // Assuming Vector2f is from Eigen
#include "Tools/Math/BHMath.h" // Assuming Angle is defined here
#include "Tools/Streams/AutoStreamable.h"

/**
 * @class DribblePath
 * A class representing the dribble path, including positions and directions.
 */
STREAMABLE(DribblePath,
{,

  /** Positions along the dribble path. */
  (std::vector<Vector2f>) stepPositions,

  /** Directions at each step along the dribble path. */
  (std::vector<Vector2f>) stepDirections,

  /** The angle associated with the dribble path. */
  (Angle) dribbleAngle,

  (float) dribbleSpeed,
});
