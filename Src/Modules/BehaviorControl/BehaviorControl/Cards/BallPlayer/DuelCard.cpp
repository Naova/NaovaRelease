/**
 * @file DuelCard.cpp
 *
 * This file implements a basic duel behavior for the code release.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

CARD(DuelCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(LookForward),
  CALLS(LookAtBall),
  CALLS(WalkToPoint),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallPlayerStrategy),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
  }),
});

class DuelCard : public DuelCardBase
{
  bool preconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::duel;
  }

  bool postconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy != BallPlayerStrategy::Strategy::duel;
  }

  option
  {
    theActivitySkill(BehaviorStatus::duel);
    Vector2f enemyAbsolute = theBallPlayerStrategy.closestEnemy.center + theRobotPose.translation; //Closest enemy absolute position
    Vector2f enemyToBall = theFieldBall.positionOnField - enemyAbsolute;       //Calculate vector between closest enemy and the ball
    initial_state(start)
    {
      transition
      {
        if((enemyToBall.squaredNorm()) < theFieldBall.positionRelative.squaredNorm()) //Check if enemy is closer to the ball than the robot
          goto Defend;
        else
        {
          goto Attack;
        }
      }
      action
      {
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    state(Attack)
    {
      transition
      {
        if(theGoToBallAndKickSkill.isDone())
          goto start;
      }
      action
      {
        float enemy_angle = (theBallPlayerStrategy.closestEnemy.center).angle();
        if(enemyToBall.angle() >= 0)
        {
          theGoToBallAndKickSkill(enemy_angle + 20_deg, KickInfo::walkForwardsLeft);

        }
        else
        {
          theGoToBallAndKickSkill(enemy_angle - 20_deg, KickInfo::walkForwardsRight);
        }
      }
    }

    state(Defend)
    {
      transition
      {
        if(theWalkToPointSkill.isDone())
          goto DefendCharge;
      }
      action
      {
        theLookAtBallSkill();
        theWalkToPointSkill(calculateGoalWallPosition(500));
      }
    }

    state(DefendCharge)
    {
      transition
      {
        if(theGoToBallAndKickSkill.isDone())
          goto start;
      }
      action
      {
        theGoToBallAndKickSkill((theBallPlayerStrategy.closestEnemy.center).angle() - 10_deg, KickInfo::walkForwardsLeft);
      }
    }

  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f)).angle();
  }

  Pose2f calculateGoalWallPosition(float wallDistance)
  {
    Vector2f goalPosition = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosCenterGoal);
    Vector2f ballPosition = theFieldBall.teamPositionOnField;
    Vector2f distanceFromBall = (goalPosition - ballPosition).normalized();
    Vector2f wallPosition = theRobotPose.inversePose*(theFieldBall.teamPositionOnField + distanceFromBall*wallDistance);

    return Pose2f(theFieldBall.positionRelative.angle(),wallPosition);
  }
};

MAKE_CARD(DuelCard);
