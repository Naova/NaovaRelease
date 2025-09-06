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
#include <array>

const double DISTANCE_PENALTY = 0.05;
const double TRIANGLE_HALF_ANGLE = 20_deg;
const double SCALING_FACTOR = 10000.0; // Utilisé pour avoir une plus grande différence entre les scores.

void BehaviorUtilities::triangularZone(std::array<Vector2f, 3>& triangle, const Vector2f& initialPosition, const Vector2f& target, double triangleHalfAngle) {
   
    
    Vector2f kickDirection = (target - initialPosition).normalized();
    Vector2f kickPerpendicularDirection(kickDirection.y(), -kickDirection.x());

    double triangleHalfBase = (target - initialPosition).norm() * std::tan(triangleHalfAngle);

    triangle[0] = initialPosition;
    triangle[1] = target + kickPerpendicularDirection * triangleHalfBase;
    triangle[2] = target - kickPerpendicularDirection * triangleHalfBase;

    
}

double BehaviorUtilities::score(const Vector2f& initialPosition, const Vector2f& target, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose)
{
    double score = 0;

    std::array<Vector2f, 3> triangle;

    triangularZone(triangle, initialPosition, target, TRIANGLE_HALF_ANGLE);
    
    
    Vector2f kickDirection = (target-initialPosition).normalized();
    
    Vector2f obstaclePosition;

    for (Obstacle obstacle : obstacles)
    {
        obstaclePosition = robotPose * obstacle.center;
        
        Vector2f triangleZone[3] = {triangle[0], triangle[1], triangle[2]};
        if (Geometry::isPointInsideConvexPolygon(triangleZone, 3, obstaclePosition))
        {   
            
            Vector2f projectionpoint = Geometry::getOrthogonalProjectionOfPointOnLine(initialPosition,kickDirection,obstaclePosition);
            score += (SCALING_FACTOR / (projectionpoint - obstaclePosition).norm());
        }
    }

    score += (initialPosition-target).norm() * DISTANCE_PENALTY;

    return score;
}

Vector2f BehaviorUtilities::bestScorePosition(const Vector2f& initialPosition, const std::vector<Vector2f>& targets, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose)
{   
    double score = std::numeric_limits<double>::max();
    Vector2f bestTarget;
    
    for(Vector2f target : targets)
    {
        double computedScore = BehaviorUtilities::score(initialPosition,target, obstacles, robotPose);
        
        if(computedScore < score)
        {
            score = computedScore;
            bestTarget = target;
        }
    }
    return bestTarget;
}

Vector2f BehaviorUtilities::bestScorePosition(std::vector<Vector2f> potentialPositions, Vector2f target, Vector2f ballPosition, std::vector<Obstacle> obstacles, Pose2f robotPose)
{   
    double score = std::numeric_limits<double>::max();
    Vector2f bestInitialPosition(0,0);
    
    for(Vector2f position : potentialPositions)
    {
        double targetScore=BehaviorUtilities::score(position, target, obstacles, robotPose);
        double passerScore=BehaviorUtilities::score(ballPosition, position, obstacles, robotPose);
        
    

        // Moyenne pour que les scores ne deviennent pas trop élevés. On peut facilement changer le poids des variables
        double computedScore = (targetScore + passerScore)/2;
        
        if(computedScore < score)
        {
            score = computedScore;
            bestInitialPosition = position;
        }
    }
    
    return bestInitialPosition;
}
Vector2f BehaviorUtilities::bestScorePositionInGoal(const Vector2f& initialBallPosition, const std::vector<Obstacle>& obstacles, const Pose2f& robotPose){
    std::vector<Vector2f> targets;

    if(Blackboard::getInstance().exists("FieldDimensions"))
    {
      const FieldDimensions& theFieldDimensions = static_cast<const FieldDimensions&>(Blackboard::getInstance()["FieldDimensions"]);
      targets.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine + 200, theFieldDimensions.yPosCenterGoal));
      targets.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine + 200, theFieldDimensions.yPosLeftGoal*0.75));
      targets.push_back(Vector2f(theFieldDimensions.xPosOpponentGroundLine + 200, theFieldDimensions.yPosRightGoal*0.75));
    }

    ASSERT(targets.size() > 0);
    double bestScore = std::numeric_limits<double>::max();
    Vector2f bestTarget;
    for(Vector2f target : targets){
        double scoreI = score(initialBallPosition, target, obstacles, robotPose);
        if(scoreI < bestScore){
            bestScore = scoreI;
            bestTarget = target;
        } 
    }

    return bestTarget;
}


