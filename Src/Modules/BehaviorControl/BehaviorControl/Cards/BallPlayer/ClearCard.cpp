/**
 * @file ClearCard.cpp
 *
 * This file implements a clear behavior when the ball is in own penalty area
 *
 * @author Christine
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

CARD(ClearCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(FieldBall),
  REQUIRES(BallPlayerStrategy),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
  }),
});

class ClearCard : public ClearCardBase
{
    bool preconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::clear;
  }

  bool postconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy != BallPlayerStrategy::Strategy::clear;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::clear);
    theGoToBallAndKickSkill(calcAngleToTarget(), KickInfo::walkForwardsLeftLong);
  }

  Angle calcAngleToTarget() const
  {
    return (theRobotPose.inversePose * theBallPlayerStrategy.targetForClear).angle();
  }
};

MAKE_CARD(ClearCard);