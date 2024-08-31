/**
 * @file KickAtGoalCard.cpp
 *
 * This file implements a basic striker behavior for the code release.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/BehaviorControl/BehaviorUtilities.h"
#include"Representations/Modeling/ObstacleModel.h"
#include"Representations/BehaviorControl/FieldBall.h"

CARD(KickAtGoalCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(SmartKick),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallPlayerStrategy),
  REQUIRES(ObstacleModel),
  REQUIRES(FieldBall),
});

class KickAtGoalCard : public KickAtGoalCardBase
{
  bool preconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::kickAtGoal;
  }

  bool postconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy != BallPlayerStrategy::Strategy::kickAtGoal;
  }

  option
  {
    theActivitySkill(BehaviorStatus::kickAtGoal);

    initial_state(goToBallAndKick)
    {
      action
      {
        theSmartKickSkill(theBallPlayerStrategy.targetForKick);
      }
    }
  }
};

MAKE_CARD(KickAtGoalCard);
