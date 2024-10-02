/**
 * @file BallPlayerStrategyProvider.h
 *
 * This file declares a module that provides the BallPlayerStrategy representation
 *
 * @author Elias
 */

#pragma once
 
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"

MODULE(BallPlayerStrategyProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotInfo),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  PROVIDES(BallPlayerStrategy),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),

  DEFINES_PARAMETERS(
  {,
    (float) (1000.f) enemyRangeBallDistance, //Max enemy distance for stating a duel
    (float)(5000.f) passRange,
    (float)(250.f) obstacleTolerance,
    (float) (3000.f) kickRange,
    (double) (30.f) offset2v2score,
  }),
});

class BallPlayerStrategyProvider : public BallPlayerStrategyProviderBase
{
  public:
    /**
     * This method is called when the representation provided needs to be updated.
     * @param theBallPlayerStrategy The representation updated.
     */
    void update(BallPlayerStrategy& theBallPlayerStrategy) override;

  private:
    /**
     * This method finds the teammate closest to the goal and withing the effective pass radius
     * Returns a vector with the kick direction we want for the pass
     */
    Vector2f findTeammateForPass();

    /**
     * This method checks if there is obstacles in the way when we want to pass to a teammate
     * Returns true if the pass won't be blocked
     */
    bool isTeammateFree(Vector2f teammate);
};
