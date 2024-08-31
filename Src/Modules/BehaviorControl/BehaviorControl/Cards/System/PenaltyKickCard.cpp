/**
 * @file PenaltyKickCard.cpp
 *
 * This file specifies the behavior for a robot during a penaltykick.
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(PenaltyKickCard,
{,
  CALLS(Activity),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) penaltyKickDeck,
  }),
});

class PenaltyKickCard : public PenaltyKickCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    dealer.deal(penaltyKickDeck)->call();
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(PenaltyKickCard);
