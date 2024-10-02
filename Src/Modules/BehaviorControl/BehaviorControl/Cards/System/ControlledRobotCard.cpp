/**
 * @file ControlledRobotCard.cpp
 *
 * This file specifies the behavior for a robot in a challenge game.
 * @author Nadir
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"

CARD(ControlledRobotCard,
{,
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  CALLS(Activity),
  CALLS(Stand),
  CALLS(LookActive),
});

class ControlledRobotCard : public ControlledRobotCardBase
{
  bool preconditions() const override
  { 
      #ifdef TARGET_ROBOT
        return theRobotInfo.number == 2;
      #else  
        return theRobotInfo.number == 1;
      #endif
  }
  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theStandSkill();
    theLookActiveSkill();
    theActivitySkill(BehaviorStatus::c2vs2);
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(ControlledRobotCard);
