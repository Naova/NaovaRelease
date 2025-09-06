/**
 * @file DribblePathProvider.h
 *
 * This file declares a module for determining optimal dribbling paths around obstacles.
 *
 * @author Aissa Bouaraguia
 */

#pragma once

#include "Representations/BehaviorControl/DribblePath.h"
#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/BehaviorControl/FieldRating.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"


MODULE(DribblePathProvider,
{,
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(FieldRating),
  PROVIDES(DribblePath),
  REQUIRES(ObstacleModel),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (float)(0.8f) dribbleSpeed,
    (int)(300) iterationSteps, /**< Num of iteration steps. */
    (float)(10.f) stepLength, /**< Iterate this much per step. */
    (float)(15.f) searchStepDrawWidth,
    (float)(5.f) searchStepDrawScale,
    (int)(25) searchStepDrawModulo,
    (float)(0.7f) minimumDribbleSpeed,
    (float)(1.0f) maximumDribbleSpeed,
    (int) (300) minimumDistance,
    (int) (4000) maximumDistance,
  }),
});

class DribblePathProvider : public DribblePathProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theDribblePath The representation updated.
   */
  void update(DribblePath& theDribblePath) override;

  float calcClosestObstacleDistance() const;
};
