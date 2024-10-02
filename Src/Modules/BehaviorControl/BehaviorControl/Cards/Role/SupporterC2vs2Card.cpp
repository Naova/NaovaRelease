/**
 * @file SupporterC2vs2Card.cpp
 *
 * This file specifies the behavior for a supporter robot.
 * @author Nadir
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(SupporterC2vs2Card,
{,
  CALLS(Annotation),
  REQUIRES(TeamBehaviorStatus),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) supporterDeck,
  }),
});

class SupporterC2vs2Card : public SupporterC2vs2CardBase
{
  bool preconditions() const override
  {
    return !theTeamBehaviorStatus.role.playsTheBall() && !theTeamBehaviorStatus.role.isGoalkeeper();
  }

  bool postconditions() const override
  {
    return theTeamBehaviorStatus.role.playsTheBall() || theTeamBehaviorStatus.role.isGoalkeeper();
  }

  void execute() override
  {
    theAnnotationSkill("Supporter");
    dealer.deal(supporterDeck)->call();
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(SupporterC2vs2Card);
