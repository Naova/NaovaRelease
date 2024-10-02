/**
 * @file AutonomousRobotCard.cpp
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

CARD(AutonomousRobotCard,
{,
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),

  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) AutonomousRobotDeck,
  }),
});
class AutonomousRobotCard : public AutonomousRobotCardBase
{
  bool preconditions() const override
  { 
      #ifdef TARGET_ROBOT
        return theRobotInfo.number == 1;
      #else  
        return theRobotInfo.number == 2;
      #endif
  }
  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    dealer.deal(AutonomousRobotDeck)->call();
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(AutonomousRobotCard);
