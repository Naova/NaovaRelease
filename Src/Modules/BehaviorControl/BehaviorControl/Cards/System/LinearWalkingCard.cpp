/**
 * @file LinearWalkingCard.cpp
 *
 * This file implements a test for the linear walking
 * 
 * @author Juan Camilo Vela
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"



CARD(LinearWalkingCard,
{,
  CALLS(Activity),
  CALLS(WalkAtRelativeSpeed),
  CALLS(LookForward),
  DEFINES_PARAMETERS(
  {,
    (float)(1) walkSpeed,
  }),
});

class LinearWalkingCard : public LinearWalkingCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return false;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::linearWalking);
    theLookForwardSkill();
    theWalkAtRelativeSpeedSkill(Pose2f(0.f,walkSpeed, 0.f));
  }
};

MAKE_CARD(LinearWalkingCard);
