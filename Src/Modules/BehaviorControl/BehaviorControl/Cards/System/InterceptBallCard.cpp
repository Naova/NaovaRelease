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
#include "Tools/Math/BHMath.h"
#include "Tools/BehaviorControl/Interception.h"
#include "Representations/Communication/RobotInfo.h"

CARD(InterceptBallCard,
{,
  CALLS(Activity),
  CALLS(InterceptBall),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
});

class InterceptBallCard : public InterceptBallCardBase
{
  bool preconditions() const override
  {
    if(theRobotInfo.isGoalkeeper()) // Keeper
    {
      return theFieldBall.isRollingTowardsOwnGoal;
    }
    else // Any other player
    {
      // Is the ball going to pass the robot?
      bool intersectsYAxis = theFieldBall.intersectionPositionWithOwnYAxis != Vector2f::Zero();

      // We shouldn't block a kick that might go into the opponent's goal
      bool shouldLetPass = theFieldBall.positionOnField.x() > 0 && theFieldBall.isRollingTowardsOpponentGoal;

      return intersectsYAxis && !shouldLetPass;
    }
  }

  bool postconditions() const override
  {
    return !preconditions();
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::interceptBall);

    if (theRobotInfo.isGoalkeeper()) // Keeper
    {
      theInterceptBallSkill(bit(Interception::genuflectStand) | bit(Interception::walk) | bit(Interception::jumpLeft) | bit(Interception::jumpRight) | bit(Interception::stand), true, true);
    }
    else // Any other player
    {
      theInterceptBallSkill(bit(Interception::walk) | bit(Interception::stand), true, false);
    }
  }
};

MAKE_CARD(InterceptBallCard);
