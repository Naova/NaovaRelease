/**
 * @file BallPlayerStrategyProvider.cpp
 *
 * This file implements a module that provides the BallPlayerStrategy representation
 *
 * @author Elias et Marc-Olivier et Nadir
 */

#include "BallPlayerStrategyProvider.h"
#include "Tools/BehaviorControl/BehaviorUtilities.h"
#include <limits>   

MAKE_MODULE(BallPlayerStrategyProvider, behaviorControl);

void BallPlayerStrategyProvider::update(BallPlayerStrategy& theBallPlayerStrategy)
{
    static const double threshold = 55.0;
    static const double margin = 10.0;
    static BallPlayerStrategy::Strategy lastStrategy = BallPlayerStrategy::Strategy::unknown;

    float closestEnemyDistance = std::numeric_limits<float>::max();
    double score = 0.0;
    double passScore = 0.0;
    Vector2f bestTarget;
    Vector2f teammate = Vector2f(0.f, 0.f);
    std::vector<Vector2f> listPoint{Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, 250),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, 500),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, -250),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, -500)};

    if(theFieldBall.positionOnField.x() < theFieldDimensions.xPosOwnPenaltyArea)
    {   
        theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::pass;
        if(theFieldBall.positionOnField.y() > 0)
            theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * Vector2f(0.f, theFieldDimensions.yPosLeftSideline * 0.8f);
        else
            theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * Vector2f(0.f, theFieldDimensions.yPosRightSideline * 0.8f);
    }
    else
    {
        for (Vector2f target : listPoint)
        {
            double computedScore = BehaviorUtilities::score(theFieldBall.positionOnField, target, theObstacleModel.obstacles, theRobotPose);

            if (computedScore > score)
            {
                score = computedScore;
                bestTarget = target;
            }
        }

        teammate = findTeammateForPass();

        if (teammate != Vector2f(0.f, 0.f))
        {
            for (Vector2f target : listPoint)
            {
                double computedScore = BehaviorUtilities::score(teammate, target, theObstacleModel.obstacles, theRobotPose);

                if (computedScore > passScore)
                {
                    passScore = computedScore;
                }
            }
        }

        if ((score > threshold || passScore > threshold) && 
            (lastStrategy == BallPlayerStrategy::Strategy::pass || lastStrategy == BallPlayerStrategy::Strategy::kickAtGoal))
        {
            if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
            {
                if (passScore > (score-offset2v2score))
                {
                    theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::pass;
                    theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * teammate;
                }
                else
                {   
                    theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::kickAtGoal;
                    theBallPlayerStrategy.targetForKick = bestTarget;
                }
            }
            else
            {
                theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::pass;
                theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * teammate;
            }

        }
        else if (score > threshold + margin || passScore > threshold + margin)
        {
            if (passScore > score)
            {
                theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::pass;
                theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * teammate;
                lastStrategy = BallPlayerStrategy::Strategy::pass;
            }
            else
            {
                theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::kickAtGoal;
                theBallPlayerStrategy.targetForKick = bestTarget;
                lastStrategy = BallPlayerStrategy::Strategy::kickAtGoal;
            }
        }
        else
        {
            if(std::abs(theFieldBall.positionOnField.y()) < theFieldDimensions.yPosLeftSideline * 0.7f)
            {
                for (const Obstacle& obstacle : theObstacleModel.obstacles)
                {
                    float obstacleRelative = obstacle.center.norm();
                    float obstacleAngle = obstacle.center.angle();
                    if (obstacle.isOpponent() && obstacleRelative < closestEnemyDistance && abs(obstacleAngle) < 80_deg)
                    {
                        closestEnemyDistance = obstacleRelative;
                        theBallPlayerStrategy.closestEnemy = obstacle;
                    }
                }
            }            

            if (closestEnemyDistance < enemyRangeBallDistance) 
            {
                theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::duel;
            }
            else
            {   
                theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::pass;
                theBallPlayerStrategy.targetForPass = theRobotPose.inversePose * teammate;
            }
            lastStrategy = theBallPlayerStrategy.currentStrategy;
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