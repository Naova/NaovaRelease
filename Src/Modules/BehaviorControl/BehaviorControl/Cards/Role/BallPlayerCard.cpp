/**
 * @file StrikerCard.cpp
 *
 * This file specifies the behavior for a ball player robot.
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(BallPlayerCard,
{,
  CALLS(Annotation),
  REQUIRES(TeamBehaviorStatus),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) ballPlayerDeck,
  }),
});

class BallPlayerCard : public BallPlayerCardBase
{
  bool preconditions() const override
  {
    return theTeamBehaviorStatus.role.playsTheBall();
  }

  bool postconditions() const override
  {
    return !theTeamBehaviorStatus.role.playsTheBall();
  }

  void execute() override
  {
    theAnnotationSkill("Ball player");
    dealer.deal(ballPlayerDeck)->call();
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(BallPlayerCard);
