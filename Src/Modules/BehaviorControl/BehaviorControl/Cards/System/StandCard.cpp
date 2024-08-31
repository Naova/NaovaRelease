/**
 * @file StandCard.cpp
 *
 * This file specifies the behavior for a standing robot.
 * 
 * @author Olivier St-Pierre
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(StandCard,
{,
  CALLS(Activity),
  CALLS(Stand),
  CALLS(LookAtBall),
});

class StandCard : public StandCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::stand);
    theLookAtBallSkill();
    theStandSkill();
  }
};

MAKE_CARD(StandCard);
