//
// Created by gortium on 15/02/19.
//

#pragma once
#include "Tools/Function.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(DribblingQLearning,
{,
    (float) (0.0) Vx,
    (float) (0.0) Vy,
    (float) (0.0) Vr,
});
