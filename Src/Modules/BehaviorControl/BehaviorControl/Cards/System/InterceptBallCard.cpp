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
    return theRobotInfo.number == 1 && theFieldDimensions.isInOwnPenaltyArea(theRobotPose.translation) && 
      theFieldBall.isRollingTowardsOwnGoal;
  }

  bool postconditions() const override
  {
    return !theFieldBall.isRollingTowardsOwnGoal;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::interceptBall);
    theInterceptBallSkill(bit(Interception::genuflectStand) | bit(Interception::walk) | bit(Interception::jumpLeft) | bit(Interception::jumpRight) | bit(Interception::stand), true, true);
  }
};

MAKE_CARD(InterceptBallCard);
