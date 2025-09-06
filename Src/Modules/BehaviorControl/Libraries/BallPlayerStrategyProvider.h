/**
 * @file BallPlayerStrategyProvider.h
 *
 * This file declares a module that provides the BallPlayerStrategy representation
 *
 * @author Aissa, Elias
 */

#pragma once


#include "Representations/BehaviorControl/DribblePath.h"
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/RobotHadBallContact.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/Settings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Modules/BehaviorControl/KickoffStateProvider/KickoffStateProvider.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Infrastructure/FrameInfo.h"


MODULE(BallPlayerStrategyProvider,
{,
  REQUIRES(RobotHadBallContact),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotInfo),
  REQUIRES(FieldBall),
  REQUIRES(DribblePath),
  REQUIRES(FieldDimensions),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(GameInfo),
  REQUIRES(KickoffState),
  REQUIRES(BallModel),
  REQUIRES(FrameInfo),
  PROVIDES(BallPlayerStrategy),
  DEFINES_PARAMETERS(
  {,

    (float) (500.f) distanceForDuel, //Max enemy distance for stating a duel
    (float)(3500.f) passRange,
    (float)(350.f) obstacleTolerance,
    (float) (3000.f) kickRange,
    (double) (45.0) scoreThreshold,
    (double) (10.0) decisionMargin,
    (double) (40_deg) angleToConsiderDuel,
    (float) (100.f) scoreDifference,
    (float) (90.f) strategyBias, // Threshold value for strategy change. (The new strategy must have a score different from 'strategyBias' to be considered)
    (float) (1500.f) goalTargetBias, // Threshold value for goal target change. (The distance between the new goal target and the old one must be at least 'goalTargetBias' to be considered)
    (float) (250.f) passTargetBias, // Threshold value for pass target change. (The distance between the new passing target and the old one must be at least 'passingTargetBias' to be considered)
    (float) (500.f) ballDifference,
    (double) (0.85f) kickWeight, // Emphasis on the kick (the smallest number has the greatest importance)
    (double) (1.15f) passWeight, // Emphasis on the pass (the smallest number has the greatest importance)
    (double) (1.f) dribbleWeight, // Emphasis on the dribble (the smallest number has the greatest importance)
    (double) (0.5f) firstDribbleIterationCoefficient, // Base value defined before the iteration in the dribble score calculation
    (double) (0.f) canShootCoefficient, //Coefficient if Ball Player can't kick the ball
    (double) (480.f) clearPositionStep,
    (double) (300.f) obstacleDistanceToLine,
    (double) (2500.f) maximumBlockingDistanceFromOpponentGoal,
    (double) (800.f) maximumBlockingDistanceFromOpponentPlayer,
    (double) (4000.f) x_axisDribbleDistanceBasis, //this distance from the middle of the field is used as a basis to allow more dribbling action
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
    
    Vector2f findTeammateForPass(Vector2f robotAbsolutePosition);

    /**
     * This method checks if there is obstacles in the way when we want to pass to a teammate
     * Returns distance(OpponentGoal, Teammate) / distance(Robot, Teammate) + totalObstacles in zone
     */
    double teammateScore(Vector2f robotAbsolutePosition, Vector2f teammateAbsolutePosition);

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
    double passingScore(const std::vector<Vector2f>& kickAtGoalOptions, Vector2f teammateAbsolutePosition);
    
    /**
     * This method checks the distance to the closest enemy
     * Returns the distance to the closest enemy
     */
    float closestEnemyToDuel(BallPlayerStrategy& theBallPlayerStrategy);

    double dribblingScore(std::vector<Vector2f> stepPositions, KickOptionInfo& kickOptionInfo, const std::vector<Vector2f>& kickAtGoalOptions);

    /**
     * This method proceeds to the action of the robot based on the scores of pass, kick and dribble 
     */
    void nextAction(double passScore, double kickScore, double dribbleScore, BallPlayerStrategy& theBallPlayerStrategy,Vector2f& teammateAbsolutePosition,KickOptionInfo& kickOptionInfo);
    
    void draw(BallPlayerStrategy& theBallPlayerStrategy) const;

    bool isBlockedNearOpponentGoal(KickOptionInfo& kickOptionInfo);
};

