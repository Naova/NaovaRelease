/**
 * @file C2vs2Card.cpp
 *
 * This file specifies the behavior for a robot in a challenge game.
 * @author Nadir
 */ 

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(C2vs2Card,
{,
  REQUIRES(TeamBehaviorStatus),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) RobotModeDeck,
  }),
});

class C2vs2Card : public C2vs2CardBase
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
        dealer.deal(RobotModeDeck)->call();
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(C2vs2Card);
