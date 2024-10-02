/**
 * @file OffensiveCard.cpp
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

CARD(OffensiveCard,
{,
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(RobotInfo),
  CALLS(Activity),
  CALLS(Stand),
  CALLS(LookActive),
  CALLS(Annotation),

  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) ballPlayerDeck,
    (DeckOfCards<CardRegistry>) supporterDeck,
    (DeckOfCards<CardRegistry>) keeperDeck,
  }),
});

class OffensiveCard : public OffensiveCardBase
{
  bool preconditions() const override
  {
    return  true;
  }

  bool postconditions() const override
  {
    return true;
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

MAKE_CARD(OffensiveCard);
