/**
 * @file BehaviorUtilities.h
 *
 * This file declares a class that represents functions that will be use in the behavior
 *
 * @author Catarina Castro
 */

#pragma once

#include "Tools/Math/Eigen.h"
#include <vector>
#include "Tools/Modeling/Obstacle.h"

class BehaviorUtilities
{
  /**
   * Computes the score between two points
   *
   * @param initialPosition The first point
   * @param target The second point.
   * @param obstacles List of known obstacles
   */
  public : 
    static double score(Vector2f initialPosition, Vector2f target, std::vector<Obstacle> obstacles, Pose2f inversePose);

/**
   * Computes the best score between a list of given targets and the robot position
   *
   * @param initialPosition The first point
   * @param target List of the possible targets.
   * @param obstacles List of known obstacles
   */
  public : 
    static Vector2f bestScorePosition(Vector2f initialPosition, std::vector<Vector2f> targets, std::vector<Obstacle> obstacles, Pose2f inversePose);
};