/**
 * @file KeeperC2vs2Card.cpp
 *
 * This file specifies the behavior for a keeper robot.
 * @author Nadir
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(KeeperC2vs2Card,
{,
  CALLS(Annotation),
  REQUIRES(TeamBehaviorStatus),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) keeperDeck,
  }),
});

class KeeperC2vs2Card : public KeeperC2vs2CardBase
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

MAKE_CARD(KeeperC2vs2Card);