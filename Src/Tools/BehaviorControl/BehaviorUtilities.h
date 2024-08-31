/**
 * @file BehaviorUtilities.h
 *
 * This file declares a class that represents functions that will be use in the behavior
 *
 * @author Catarina Castro et Olivier St-Pierre
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include <vector>
#include "Tools/Modeling/Obstacle.h"
#include "Representations/Communication/RobotInfo.h"

class BehaviorUtilities
{
  public:
    static double score(const Vector2f& initialPosition, const Vector2f& target, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose);

    static Vector2f bestScorePosition(const Vector2f& initialPosition, const std::vector<Vector2f>& targets, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose);
    static Vector2f bestScorePosition(std::vector<Vector2f> potentialPositions, Vector2f target, Vector2f ballPosition, 
                                      std::vector<Obstacle> obstacles, Pose2f robotPose);

    static Vector2f bestScorePositionInGoal(const Vector2f& initialPosition, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose);

};