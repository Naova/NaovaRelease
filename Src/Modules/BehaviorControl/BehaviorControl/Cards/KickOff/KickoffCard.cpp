/**
 * @file KickoffCard.cpp
 *
 * This file specifies the behavior for a robot during Kickoff.
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/BehaviorControl/KickoffState.h"

CARD(KickoffCard,
{,
  REQUIRES(KickoffState),
  CALLS(Activity),
  CALLS(Stand),
  CALLS(LookAtGlobalBall),
});

class KickoffCard : public KickoffCardBase
{
  bool preconditions() const override
  {
    return !theKickoffState.allowedToEnterCenterCircle;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::kickOffWait);
    theStandSkill();
    theLookAtGlobalBallSkill();
  }
};

MAKE_CARD(KickoffCard);
