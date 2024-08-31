/**
 * @file PassCard.cpp
 *
 * This file implements a pass behavior for the striker
 *
 * @author Elias et Marc-Olivier
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"

CARD(PassCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(FieldBall),
  REQUIRES(BallPlayerStrategy),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
  }),
});

class PassCard : public PassCardBase
{
    bool preconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::pass;
  }

  bool postconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy != BallPlayerStrategy::Strategy::pass;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::pass);
    theGoToBallAndKickSkill(calcAngleToTarget(), KickInfo::walkForwardsLeftLong);
  }

  Angle calcAngleToTarget() const
  {
    return theBallPlayerStrategy.targetForPass.angle();
  }
};

MAKE_CARD(PassCard);