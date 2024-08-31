/**
 * @file SupporterCard.cpp
 *
 * This file specifies the behavior for a supporter robot.
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(SupporterCard,
{,
  CALLS(Annotation),
  REQUIRES(TeamBehaviorStatus),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) supporterDeck,
  }),
});

class SupporterCard : public SupporterCardBase
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

MAKE_CARD(SupporterCard);
