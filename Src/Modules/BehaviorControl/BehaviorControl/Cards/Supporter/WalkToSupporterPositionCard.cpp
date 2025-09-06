/**
 * @file WalkToSupporterPositionCard.cpp
 *
 * This file specifies the behavior for the supporter when he walks to his position.
 * 
 * @author Olivier St-Pierre
 * @author Jérémy Thu-Thon
 *
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/SupporterPositioning.h"
#include "Representations/BehaviorControl/DynamicSupporterPositionning.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/GameInfo.h"


CARD(WalkToSupporterPositionCard,
{,
  REQUIRES(DynamicSupporterPositionning),
  REQUIRES(SupporterPositioning),
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  REQUIRES(GameInfo),
  CALLS(Activity),
  CALLS(WalkToPoint),
  CALLS(LookActive),
});

class WalkToSupporterPositionCard : public WalkToSupporterPositionCardBase
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
    theActivitySkill(BehaviorStatus::supporterPositioning);
    theLookActiveSkill(true);
    Pose2f position;
    if (theGameInfo.setPlay == SET_PLAY_NONE && theSupporterPositioning.basePose.translation.x() > 0)
    {
      position = theDynamicSupporterPositionning.basePose;
    }
    else
    {
      position = theSupporterPositioning.basePose;
    }
    
    Pose2f relativeTarget = theRobotPose.inversePose * position;
    relativeTarget = Pose2f(theFieldBall.teamPositionRelative.angle(), relativeTarget.translation.x(), relativeTarget.translation.y());
    theWalkToPointSkill(relativeTarget, /* speed: */1.f, /* rough: */ true, /* disableObstacleAvoidance: */ false, /* disableAligning: */true);
  }
};

MAKE_CARD(WalkToSupporterPositionCard);
