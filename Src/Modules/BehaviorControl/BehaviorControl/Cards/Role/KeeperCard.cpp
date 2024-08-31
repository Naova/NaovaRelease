/**
 * @file KeeperCard.cpp
 *
 * This file specifies the behavior for a keeper robot.
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(KeeperCard,
{,
  CALLS(Annotation),
  REQUIRES(TeamBehaviorStatus),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) keeperDeck,
  }),
});

class KeeperCard : public KeeperCardBase
{
  bool preconditions() const override
  {
    return theTeamBehaviorStatus.role.isGoalkeeper();
  }

  bool postconditions() const override
  {
    return !theTeamBehaviorStatus.role.isGoalkeeper();
  }

  void execute() override
  {
    theAnnotationSkill("Keeper");
    dealer.deal(keeperDeck)->call();
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(KeeperCard);
