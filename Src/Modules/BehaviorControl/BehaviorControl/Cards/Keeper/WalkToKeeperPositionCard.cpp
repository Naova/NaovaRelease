/**
 * @file WalkToKeeperPositionCard.cpp
 *
 * This file implements a basic keeper behavior.
 *
 * @author Manu et Paul
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Tools/Math/BHMath.h"

CARD(WalkToKeeperPositionCard,
{,
  CALLS(Activity),
  CALLS(WalkToPoint),
  CALLS(LookActive),
  CALLS(LookAtGlobalBall),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(5000) ballNotSeenTimeout,
    (float)(0.02f) reductionY,
    (float)(0.7f) radius,
  }),
});

class WalkToKeeperPositionCard : public WalkToKeeperPositionCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::alignToBall);

    initial_state(start)
    {
      transition
      {
        if (theFieldBall.teamBallWasSeen(ballNotSeenTimeout) && !theFieldDimensions.isInsideOwnHalf(theFieldBall.teamPositionOnField))
          goto alignToBall;
      }

      action
      {
        if(theFieldBall.teamPositionOnField.x() > theFieldDimensions.xPosOwnGroundLine/2)
          theLookActiveSkill(true);
        else
          theLookAtGlobalBallSkill();

        theWalkToPointSkill(getInitPos(), walkSpeed, /* rough: */ true, /* disableObstacleAvoidance: */ true, true);
      }
    }

    state(alignToBall)
    {
      transition
      {
        if(!theFieldBall.teamBallWasSeen(ballNotSeenTimeout) || theFieldDimensions.isInsideOwnHalf(theFieldBall.teamPositionOnField))
          goto start;
      }

      action
      {
        auto targetPos = getAlignVector();
        theLookActiveSkill(true);
        theWalkToPointSkill(Pose2f(calcAngleToBall(), targetPos), walkSpeed, /* rough: */ true, /* disableObstacleAvoidance: */ true, /* disableAligning: */true);
      }
    }
  }

  Angle calcAngleToBall() const
  {
    return (theRobotPose.inversePose * theFieldBall.teamPositionOnField).angle();
  }

  Angle calcAngleToCenterField() const
  {
    return (theRobotPose.inversePose *  Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosCenterGoal)).angle();
  }

  Vector2f getAlignVector() const
  {
    Vector2f centerGoal = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldBall.teamPositionOnField.y() * reductionY);
    Vector2f vector = theFieldBall.teamPositionOnField - centerGoal;
    Vector2f vectorNormalized = vector.normalized();
    Vector2f position = (centerGoal + (vectorNormalized * (std::abs(theFieldDimensions.yPosLeftGoal) * radius)));

    return theRobotPose.inversePose * position;
  }

  Pose2f getInitPos() const
  {
    return Pose2f(calcAngleToCenterField(),theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundLine + theRobotDimensions.footLength * 1.5f, theFieldDimensions.yPosCenterGoal));
  }
};

MAKE_CARD(WalkToKeeperPositionCard);
