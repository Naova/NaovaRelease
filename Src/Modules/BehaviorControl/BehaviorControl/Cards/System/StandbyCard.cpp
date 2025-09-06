/**
 * @file StandbyCard.cpp
 *
 * This file specifies the behavior for a robot in the Standby game state.
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

CARD(StandbyCard,
{,
  CALLS(Activity),
  CALLS(LookAtAngles),
  CALLS(Stand),
  CALLS(Say),
  CALLS(LookAtReferee),
  REQUIRES(GameInfo),
});

class StandbyCard : public StandbyCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_STANDBY;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_STANDBY;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::standby);
    theStandSkill(/* high: */ true);
    theSaySkill("Standby");
    theLookAtRefereeSkill();
  }
};

MAKE_CARD(StandbyCard);
