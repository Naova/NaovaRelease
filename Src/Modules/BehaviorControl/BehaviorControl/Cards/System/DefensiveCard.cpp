/**
 * @file DefensiveCard.cpp
 *
 * This file specifies the behavior for a robot in a challenge game.
 * @author Nadir
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/RobotInfo.h"

CARD(DefensiveCard,
{,
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(OwnTeamInfo),

  CALLS(Activity),
  CALLS(Stand),
  CALLS(LookActive),
  CALLS(Annotation),

  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) keeperDeck,
    (DeckOfCards<CardRegistry>) ballPlayerDeck,
    (DeckOfCards<CardRegistry>) supporterDeck,
  }),
});

class DefensiveCard : public DefensiveCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber;
  }

  bool postconditions() const override
  {
    return theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber;
  }

  void execute() override
  {
    switch(theTeamBehaviorStatus.role.role){
      case PlayerRole::RoleType::ballPlayer:
        dealer.deal(ballPlayerDeck)->call();
        setState("ballPlayer");
        break;
      case PlayerRole::RoleType::goalkeeper:
        dealer.deal(keeperDeck)->call();
        setState("goalkeeper");
        break;
      default:
        dealer.deal(supporterDeck)->call();
        setState("supporter");
        break;
    }    
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(DefensiveCard);
