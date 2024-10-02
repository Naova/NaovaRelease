/**
 * @file WalkToSupporterPositionC2vs2Card.cpp
 *
 * This file specifies the behavior for the supporter when he walks to his position.
 * 
 * @author Nadir
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/SupporterPositioning.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
#include "Representations/BehaviorControl/FieldBall.h"

CARD(WalkToSupporterPositionC2vs2Card,
{,
  REQUIRES(SupporterPositioning),
  REQUIRES(RobotPose),
  REQUIRES(FieldBall),
  CALLS(Activity),
  CALLS(LookAtGlobalBall),
  CALLS(WalkToPoint),
});

class WalkToSupporterPositionC2vs2Card : public WalkToSupporterPositionC2vs2CardBase
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
    theLookAtGlobalBallSkill();
    Pose2f relativeTarget = theFieldBall.teamPositionRelative;
    if(theFieldBall.teamPositionOnField.x()>=0 && theFieldBall.teamPositionOnField.y()>=0)
    {
      relativeTarget = Pose2f(0, relativeTarget.translation.x()-1500, relativeTarget.translation.y()-1500);
    }
    else if (theFieldBall.teamPositionOnField.x()>=0 && theFieldBall.teamPositionOnField.y()<0)
    {
      relativeTarget = Pose2f(0, relativeTarget.translation.x()-1500, relativeTarget.translation.y()+1500);
    }
    else if (theFieldBall.teamPositionOnField.x()<0 && theFieldBall.teamPositionOnField.y()<0)
    {
      relativeTarget = Pose2f(0, relativeTarget.translation.x()-1500, relativeTarget.translation.y()+1500);
    }
    else
    {
      relativeTarget = Pose2f(0, relativeTarget.translation.x()-1500, relativeTarget.translation.y()-1500);
    }
    theWalkToPointSkill(relativeTarget); 
  }
};

MAKE_CARD(WalkToSupporterPositionC2vs2Card);