
/**
 * @file BehaviorUtilities.cpp
 *
 * This file implements the module that describes the functions used for the robot behavior.
 *
 * @author Catarina Castro
 */

#include "BehaviorUtilities.h"
#include "Tools/Math/Geometry.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Debugging/Debugging.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Representations/Communication/GameInfo.h"
#include "iostream"

double BehaviorUtilities::score(const Vector2f& initialPosition, const Vector2f& target, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose)
{
    const double angle = 20_deg;
    Vector2f kickDirection = (target-initialPosition).normalized();
    Vector2f perpDirection(kickDirection.y(),-kickDirection.x());

    double score = 100.0;
    double h = ((target - initialPosition).norm() * std::tan(angle));

    Vector2f point1=target+perpDirection*h;
    Vector2f point2=target-perpDirection*h;
    Vector2f triangle[] = { initialPosition, point1, point2 };
    Vector2f obstaclePos;

    for (Obstacle obstacle : obstacles)
    {
        obstaclePos = Vector2f((obstacle.center.x() * std::cos(robotPose.rotation) - obstacle.center.y() * std::sin(robotPose.rotation)) + robotPose.translation.x(),
                         (obstacle.center.x() * std::sin(robotPose.rotation) + obstacle.center.y() * std::cos(robotPose.rotation)) + robotPose.translation.y());
        
        if (Geometry::isPointInsideConvexPolygon(triangle, 3, obstaclePos))
        {           
            Vector2f projectionpoint = Geometry::getOrthogonalProjectionOfPointOnLine(initialPosition,kickDirection,obstaclePos);
            score -= (3000.0 / (projectionpoint - obstaclePos).norm());
        }
    }

    score -= (initialPosition-target).norm() * 0.01f;
    
    return score;
}

Vector2f BehaviorUtilities::bestScorePosition(const Vector2f& initialPosition, const std::vector<Vector2f>& targets, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose)
{   
    double score = std::numeric_limits<double>::lowest();
    Vector2f bestTarget;
    
    for(Vector2f target : targets)
    {
        double computedScore = BehaviorUtilities::score(initialPosition,target,obstacles, robotPose);
        
        if(computedScore > score)
        {
            score = computedScore;
            bestTarget = target;
        }
    }
    return bestTarget;
}

Vector2f BehaviorUtilities::bestScorePosition(std::vector<Vector2f> potentialPositions, Vector2f target, Vector2f ballPosition, 
std::vector<Obstacle> obstacles, Pose2f robotPose)
{   
    double score = std::numeric_limits<double>::lowest();
    Vector2f bestInitialPosition(0,0);
    
    for(Vector2f position : potentialPositions)
    {
        double targetScore=BehaviorUtilities::score(position, target, obstacles, robotPose);
        double passerScore=BehaviorUtilities::score(ballPosition, position, obstacles, robotPose);
        
        //moyenne pour ne pas que les scores deviennent trop élevés. On peut facilement changer le poids des variables
        double computedScore = (targetScore + passerScore)/2;
        
        if(computedScore > score)
        {
            score = computedScore;
            bestInitialPosition = position;
        }
    }

    return bestInitialPosition;
}
Vector2f BehaviorUtilities::bestScorePositionInGoal(const Vector2f& initialPosition, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose){
    std::vector<Vector2f> targets;

    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      targets.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosCenterGoal));
      targets.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoal*0.75));
      targets.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoal*0.75));
    }

    ASSERT(targets.size() > 0);
    double bestScore = -1.0;
    Vector2f bestTarget;
    for(Vector2f target : targets){
        double scoreI = score(initialPosition, target, obstacles, robotPose);
        if(scoreI > bestScore){
            bestScore = scoreI;
            bestTarget = target;
        } 
    }

    return bestTarget;
}