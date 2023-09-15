/**
 * @file GoalAreaPerceptor.h
 * Provides GoalArea.
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/Odometer.h"
#include "Representations/Perception/FieldPercepts/FieldLines.h"
#include "Representations/Perception/FieldPercepts/FieldLineIntersections.h"
#include "Representations/Perception/FieldPercepts/PenaltyMarkPercept.h"
#include "Representations/Perception/FieldFeatures/GoalArea.h"
#include "Representations/Perception/FieldFeatures/FieldRelations.h"
#include "Tools/Math/BHMath.h"

MODULE(GoalAreaPerceptor,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(FieldLines),
  REQUIRES(FieldLineIntersections),
  REQUIRES(IntersectionRelations),
  REQUIRES(Odometer),
  REQUIRES(PenaltyMarkPercept),
  PROVIDES(GoalArea),
  LOADS_PARAMETERS(
  {,
    (bool)(true) usePenaltyMark,
    (float)(110.f) thresholdDisVaranzToPenaltyMark,
    (float)(750.f) maxDistVarianceOfLineEnds,
    (float)(70.f) thresholdIntersections,
    (Angle)(20_deg) thesholdAngleDisForIntersections,
    (int)(30) maxTimeOffset,
    (float)(100.f) thresholdYVarianceIntersection,
  }),
});

class GoalAreaPerceptor : public GoalAreaPerceptorBase
{
  void update(GoalArea& goalArea);

private:
  unsigned lastFrameTime = 1;
  PenaltyMarkPercept theLastPenaltyMarkPercept;

  bool searchWithPMarkAndLine(GoalArea& goalArea) const;
  bool searchWithIntersections(GoalArea& goalArea) const;

  bool checkLineDistanceToNearesPoint(const Pose2f& posePA, const Vector2f& p1, const Vector2f& p2) const;
};
