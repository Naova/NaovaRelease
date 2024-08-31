/**
 * @file OwnFreeKickCard.cpp
 *
 * This file specifies the behavior while a free kick for us is happening.
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/BehaviorControl/SupporterPositioning.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"

CARD(OwnFreeKickCard,
{,
  CALLS(Annotation),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(RobotPose),
  REQUIRES(SupporterPositioning),
  REQUIRES(FieldBall),
  CALLS(Activity),
  CALLS(LookAtGlobalBall),
  CALLS(WalkToPoint),
  LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) ballPlayerDeck,
    (DeckOfCards<CardRegistry>) supporterDeck,
  }),
});

class OwnFreeKickCard : public OwnFreeKickCardBase
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
    theAnnotationSkill("FreeKick");
    if(theTeamBehaviorStatus.role.playsTheBall()){
      dealer.deal(ballPlayerDeck)->call();
    }else{
      dealer.deal(supporterDeck)->call();
    }
      
  }
  PriorityListDealer dealer; /**< The dealer which selects the card to play. */
};

MAKE_CARD(OwnFreeKickCard);
