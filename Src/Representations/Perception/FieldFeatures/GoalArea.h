/**
 * @file GoalArea.h
 * Declaration of a struct that represents a goal Area.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "FieldFeature.h"

/**
 * @struct GoalArea
 * It defines a the pose of the goal area in relative field coordinates to the robot.
 * The goal area pose: position => middle of the area; rotation => looking in direction of goal
 */
STREAMABLE_WITH_BASE(GoalArea, FieldFeature,
{
  void draw() const;
  CHECK_FIELD_FEATURE_POSE_OF("GoalArea");

  GoalArea() = default;
  GoalArea(const Pose2f& pose) : FieldFeature(pose) {};

  /**
   * Assignment operator for Pose2f objects.
   * @param other A Pose2f object
   * @return A reference to the object after the assignment
     */
  const GoalArea& operator=(const Pose2f& other)
  {
    static_cast<Pose2f&>(*this) = other;
    // validity and co are not set
    return *this;
  };

  /**
   * Returns 1 of the 2 global position of this feature (in case of isValid == true).
   */
  const Pose2f getGlobalFeaturePosition() const override,
});
