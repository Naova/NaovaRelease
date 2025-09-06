/**
 * @file DribbleCard.cpp
 *
 * This file implements the dribble behavior
 *
 * @author Elias et Marc-Olivier
 */

#include "Representations/BehaviorControl/DribblePath.h"
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

CARD(DribbleCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndDribble),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(BallPlayerStrategy),
  REQUIRES(DribblePath),
  REQUIRES(ObstacleModel),

});

class DribbleCard : public DribbleCardBase
{
  void preProcess() override
  {
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:direction", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:step", "drawingOnField");
  }

  bool preconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::dribble;
  }

  bool postconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy != BallPlayerStrategy::Strategy::dribble;
  }

  
  void execute() override
  {
    theActivitySkill(BehaviorStatus::dribble);

    const Angle dribbleAngle = theDribblePath.dribbleAngle;
  
    float dribbleSpeed = theDribblePath.dribbleSpeed;
    theGoToBallAndDribbleSkill(dribbleAngle, false, dribbleSpeed, false, false);
  }

};

MAKE_CARD(DribbleCard);
