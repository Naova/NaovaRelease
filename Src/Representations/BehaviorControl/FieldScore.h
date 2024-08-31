/**
 * @file FieldScore.h
 *
 * Contains information about where the ball should be played to, with multiple possibilities
 *
 * @author Olivier St-Pierre
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Function.h"

STREAMABLE(FieldScore,
{,
  (std::vector<double>) scores,
});
