/**
 * @file DuelCard.cpp
 *
 * This file implements a basic duel behavior for the code release.
 *
 * @author Mathieu Gagnon
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
    (float)(15_deg) duelAngle,
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

  void execute() override
  {
    theActivitySkill(BehaviorStatus::duel);
    Vector2f enemyAbsolute = theRobotPose * theBallPlayerStrategy.closestEnemy.center; //Closest enemy absolute position
    Vector2f enemyToBall = theFieldBall.positionOnField - enemyAbsolute;       //Calculate vector between closest enemy and the ball
    float enemy_angle = (theBallPlayerStrategy.closestEnemy.center).angle();
    if(enemyToBall.angle() >= 0)
    {
      theGoToBallAndKickSkill(enemy_angle + duelAngle, KickInfo::walkForwardsLeft);
    }
    else
    {
      theGoToBallAndKickSkill(enemy_angle - duelAngle, KickInfo::walkForwardsRight);
    }
  }
};

MAKE_CARD(DuelCard);
