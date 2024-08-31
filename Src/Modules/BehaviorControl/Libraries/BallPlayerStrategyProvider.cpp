/**
 * @file BallPlayerStrategyProvider.cpp
 *
 * This file implements a module that provides the BallPlayerStrategy representation
 *
 * @author Elias, Marc-Olivier et Catarina
 */

#include "BallPlayerStrategyProvider.h"
#include "Tools/BehaviorControl/BehaviorUtilities.h"
#include <limits>   

MAKE_MODULE(BallPlayerStrategyProvider, behaviorControl);

void BallPlayerStrategyProvider::update(BallPlayerStrategy& theBallPlayerStrategy)
{
    Vector2f teammate = Vector2f(0.f, 0.f);
    KickOptionInfo kickOptionInfo;
    KickOptionInfo lastKickOptionInfo;
    Vector2f initalBallPosition = Vector2f(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    float closestEnemyDistance = closestEnemyToDuel(theBallPlayerStrategy);
    

    if((initalBallPosition - theFieldBall.positionOnField).norm() > ballDifference)
    {
        lastKickOptionInfo.kickScore = std::numeric_limits<double>::lowest();
    }
    if (!isClearingTheBall(theBallPlayerStrategy))
    {
        if (closestEnemyDistance < distanceForDuel) 
        {
            theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::duel;
            lastStrategy = theBallPlayerStrategy.currentStrategy;
        }
        else
        {
            kickOptionInfo = findBestKickTarget(kickAtGoalOptions);
            if(!(kickOptionInfo.kickScore > (lastKickOptionInfo.kickScore + scoreDifference)))
            {
                kickOptionInfo = lastKickOptionInfo;
            }
            else
            {
                lastKickOptionInfo = kickOptionInfo;                                                
            }
            teammate = findTeammateForPass();
    
            double passScore = isPassWorth(teammate, kickAtGoalOptions);

            bool isKickingOrPassing = isKickingOrPassingBall(theBallPlayerStrategy, kickOptionInfo, teammate, passScore);
            
            if (!isKickingOrPassing)
            {
                theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::dribble;
                lastStrategy = theBallPlayerStrategy.currentStrategy;
            }
            else
            {
                initalBallPosition = theFieldBall.positionOnField;
            }
        }
    }
}


Vector2f BallPlayerStrategyProvider::findTeammateForPass()
{
    Vector2f teammatePos;
    Vector2f teammatePosRelative;
    Vector2f target = Vector2f(theFieldDimensions.xPosOwnFieldBorder, 0.f); // default target so that any available target gets chosen instead

    for(auto const& teammate : theTeamData.teammates)
    {
        teammatePos = Vector2f(teammate.theRobotPose.translation.x(), teammate.theRobotPose.translation.y()); 
        teammatePosRelative = theRobotPose.inversePose * teammatePos;

        if (teammate.status != Teammate::PENALIZED && teammate.status != Teammate::FALLEN)
        {
            if (teammatePosRelative.norm() <= passRange && isTeammateFree(teammatePosRelative) && teammatePos.x() > target.x())
            {
                target = teammatePos;
            }
        }
    }

    if(target == Vector2f(theFieldDimensions.xPosOwnFieldBorder, 0.f))
    {
        return Vector2f(0.f, 0.f);
    }
    else
    {
        return target;
    }
}

bool BallPlayerStrategyProvider::isTeammateFree(Vector2f target)
{
    float angleToObstacle;
    float dotProd;
    double obstacleHeight;

    for(const Obstacle& obstacle : theObstacleModel.obstacles)
    {
        if (obstacle.type != Obstacle::teammate)
        {
            dotProd = target.x() * obstacle.center.x() + target.y() * obstacle.center.y();

            if(dotProd > 0.f && obstacle.center.norm() < (target.norm() + obstacleTolerance))
            {
                angleToObstacle = std::acos(dotProd / (target.norm() * obstacle.center.norm()));
                obstacleHeight = std::abs(sin(angleToObstacle) * obstacle.center.norm());

                if(obstacleHeight < obstacleTolerance)
                {
                    return false;
                }
            }
        }
    }

    return true;
}

bool BallPlayerStrategyProvider::isClearingTheBall(BallPlayerStrategy& theBallPlayerStrategy)
{
    //If the ball is inside the our penalty area, we clear the ball
    if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea)
    {
        theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::pass;
        if(theFieldBall.positionOnField.y() > 0)
            theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * clearingPositionLeft;
        else
            theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * clearingPositionRight;
        
        return true;
    }
    return false;
}

BallPlayerStrategyProvider::KickOptionInfo BallPlayerStrategyProvider::findBestKickTarget(const std::vector<Vector2f>& kickAtGoalOptions)
{
    KickOptionInfo kickOptionInfo;        
    kickOptionInfo.kickScore = -1.0;
    
    for(const Vector2f &target : kickAtGoalOptions)
    {
        double computedScore = BehaviorUtilities::score(theFieldBall.positionOnField, target, theObstacleModel.obstacles, theRobotPose);

        if (computedScore > kickOptionInfo.kickScore)
        {
            kickOptionInfo.kickScore = computedScore;
            kickOptionInfo.bestTarget = target;
        }
    }

    return kickOptionInfo;
}

double BallPlayerStrategyProvider::isPassWorth(Vector2f teammate, const std::vector<Vector2f>& kickAtGoalOptions)
{
    double passScore = -1.0;

    if (teammate != Vector2f::Zero())
    {
        for(const Vector2f &target : kickAtGoalOptions)
        {
            double computedScore = BehaviorUtilities::score(teammate, target, theObstacleModel.obstacles, theRobotPose);

            if (computedScore > passScore)
            {
                passScore = computedScore;
            }
        }
    }
    return passScore;
}

bool BallPlayerStrategyProvider::isKickingOrPassingBall(BallPlayerStrategy& theBallPlayerStrategy, KickOptionInfo& kickOptionInfo, Vector2f& teammate, double passScore)
{
    if ((kickOptionInfo.kickScore > scoreThreshold || passScore > scoreThreshold) && 
        (lastStrategy == BallPlayerStrategy::Strategy::pass || lastStrategy == BallPlayerStrategy::Strategy::kickAtGoal))
    {
        if (passScore > (kickOptionInfo.kickScore + scoreBias))
        {
            theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::pass;
            theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * teammate;
        }
        else
        {
            theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::kickAtGoal;
            theBallPlayerStrategy.targetForKick = kickOptionInfo.bestTarget;
        }
        return true;
    }
    else if (kickOptionInfo.kickScore > scoreThreshold + decisionMargin || passScore > scoreThreshold + decisionMargin)
    {
        if (passScore > (kickOptionInfo.kickScore + scoreBias))
        {
            theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::pass;
            theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * teammate;
            lastStrategy = BallPlayerStrategy::Strategy::pass;
        }
        else
        {
            theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::kickAtGoal;
            theBallPlayerStrategy.targetForKick = kickOptionInfo.bestTarget;
            lastStrategy = BallPlayerStrategy::Strategy::kickAtGoal;
        }
        return true;
    }
    return false;
}

float BallPlayerStrategyProvider::closestEnemyToDuel(BallPlayerStrategy& theBallPlayerStrategy)
{       
    float closestEnemyDistance = std::numeric_limits<float>::max();

    if(std::abs(theFieldBall.positionOnField.y()) < theFieldDimensions.yPosLeftSideline * 0.7f && std::abs(theFieldBall.positionOnField.x()) < theFieldDimensions.xPosOpponentGoalArea)
    {
        for(const Obstacle& obstacle : theObstacleModel.obstacles)
        {
            float obstacleDistance = obstacle.center.norm();
            Vector2f obstacleToball = (theRobotPose * obstacle.center) - theFieldBall.positionOnField;
            Vector2f kickDirectionToBall = (kickDirection - theFieldBall.positionOnField);
            float obstacleToKickAngle = kickDirectionToBall.angleTo(obstacleToball);

            if (obstacle.isOpponent() && obstacleDistance < closestEnemyDistance && abs(obstacleToKickAngle) < angleToConsiderDuel)
            {
                closestEnemyDistance = obstacleDistance;
                theBallPlayerStrategy.closestEnemy = obstacle;
            }
        }
    }
    return closestEnemyDistance;
}