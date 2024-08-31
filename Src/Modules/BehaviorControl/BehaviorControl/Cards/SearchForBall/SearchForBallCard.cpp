/**
 * @file SearchForBallCard.cpp
 *
 * This file implements a search for ball.
 *
 * @author Marc-Olivier
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Communication/RobotInfo.h"

CARD(SearchForBallCard,
{,
  CALLS(Activity),
  CALLS(SearchBall),
  CALLS(LookActive),
  REQUIRES(FieldBall),
  REQUIRES(RobotInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(5000) ballNotSeenTimeout,
  }),
});

class SearchForBallCard : public SearchForBallCardBase
{
  bool preconditions() const override
  {
    return !theFieldBall.teamBallWasSeen(ballNotSeenTimeout) && theRobotInfo.number != 1;
  }

  bool postconditions() const override
  {
    return theFieldBall.teamBallWasSeen(ballNotSeenTimeout);
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::searchForBall);
    theLookActiveSkill();
    theSearchBallSkill();
  }
};

MAKE_CARD(SearchForBallCard);
