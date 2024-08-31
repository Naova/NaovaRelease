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

MODULE(BallPlayerStrategyProvider,
{,
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotInfo),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  PROVIDES(BallPlayerStrategy),
  DEFINES_PARAMETERS(
  {,
    (float) (1000.f) distanceForDuel, //Max enemy distance for stating a duel
    (float)(2500.f) passRange,
    (float)(250.f) obstacleTolerance,
    (float) (3000.f) kickRange,
    (double) (45.0) scoreThreshold,
    (double) (10.0) decisionMargin,
    (double) (40_deg) angleToConsiderDuel,
    (float) (10.f) scoreDifference,
    (float) (5.f) scoreBias,
    (float) (500.f) ballDifference,
    (BallPlayerStrategy::Strategy) (BallPlayerStrategy::Strategy::unknown) lastStrategy,
    
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

    struct KickOptionInfo {
      double kickScore;
      Vector2f bestTarget;
    };

    // Positions in goal to kick at.
    const std::vector<Vector2f> kickAtGoalOptions{Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, 250),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, 500),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, -250),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, -500)};

    const Vector2f clearingPositionLeft = Vector2f(0.f, theFieldDimensions.yPosLeftSideline * 0.8f);
    const Vector2f clearingPositionRight = Vector2f(0.f, theFieldDimensions.yPosRightSideline * 0.8f);
    const Vector2f kickDirection = Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosCenterGoal);
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

    /**
     * This method checks if we need to clear the ball
     * Returns true if we need to clear the ball
     */
    bool isClearingTheBall(BallPlayerStrategy& theBallPlayerStrategy);

    /**
     * This method finds the best target for a kick
     * Returns a struct with the best target and the score of the kick
     */
    KickOptionInfo findBestKickTarget(const std::vector<Vector2f>& kickAtGoalOptions);

    /**
     * This method calculates the score of a pass
     * Returns the score of the pass
     */
    double isPassWorth(Vector2f teammate, const std::vector<Vector2f>& kickAtGoalOptions);

    /**
     * This method checks if the robot is kicking or passing the ball
     * Returns true if the robot is kicking or passing
     */
    bool isKickingOrPassingBall(BallPlayerStrategy& theBallPlayerStrategy, KickOptionInfo& kickOptionInfo, Vector2f& teammate, double passScore);

    /**
     * This method checks the distance to the closest enemy
     * Returns the distance to the closest enemy
     */
    float closestEnemyToDuel(BallPlayerStrategy& theBallPlayerStrategy);
};
