//
// Created by vincent on 19-05-11.
//

#pragma once

#include "Tools/Math/Pose2f.h"
#include "Tools/Math/Pose3f.h"
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Function.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Modules/MotionControl/Balancer/LIPStateEstimator.h"
#include "Modules/MotionControl/Balancer/ZmpController.h"


STREAMABLE(DynamicTesting,
{
    /**
     * Initializes the generator. Must be called whenever control over stabilization is handed over to this module
     * Must also be called once after creation.
     * @return current zmp with body rotation as rotation matrix. Hand over to balancing methods to avoid jumps. can be slowly set to zero.
     */
    FUNCTION(Pose3f()) init;

    /**
     * Modifies joint angles to make the robot stand stable
     * @param jointRequest The requested joint angles, which will be modified
     * @param zmpPreviewsX current and future zmpPositions (X-Value)
     * @param zmpPreviewsY current and future zmpPositions (Y-Value)
     * @return returns true when a stable one-legged position is reached
     */
    FUNCTION(bool(JointAngles& jointAngles)) addBalance;
    ,
});

