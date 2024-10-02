/**
 * @file BehaviorUtilities.cpp
 *
 * This file implements the module that describes the functions used for the robot behavior.
 *
 * @author Catarina Castro
 */

#include "BehaviorUtilities.h"
#include "Tools/Math/Geometry.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"

double BehaviorUtilities::score(Vector2f initialPosition, Vector2f target, std::vector<Obstacle> obstacles, Pose2f inversePose)
{
    const double angle = 15_deg;
    const double positionBoost = 1.25;

    double score = 100.0;
    double h = ((target - initialPosition).norm() * std::tan(angle));

    Vector2f point1(target.x() * positionBoost, (target.y() + h) * positionBoost);
    Vector2f point2(target.x() * positionBoost, (target.y() - h) * positionBoost);
    Vector2f triangle[] = { initialPosition, point1, point2 };
    Vector2f obstaclePos;

    for (Obstacle obstacle : obstacles)
    {
        obstaclePos = Vector2f((obstacle.center.x() * std::cos(inversePose.rotation) - obstacle.center.y() * std::sin(inversePose.rotation)) + inversePose.translation.x(),
                         (obstacle.center.x() * std::sin(inversePose.rotation) + obstacle.center.y() * std::cos(inversePose.rotation)) + inversePose.translation.y());
        
        if (Geometry::isPointInsideConvexPolygon(triangle, 3, obstaclePos))
        {           
            Vector2f projectionpoint = Geometry::getOrthogonalProjectionOfPointOnLine(initialPosition,target.normalized(),obstaclePos);
            
            score -= (3000.0 / (projectionpoint - obstaclePos).norm());
        }
    }

    score -= (initialPosition-target).norm() * 0.01f;

    return score;
}

Vector2f BehaviorUtilities::bestScorePosition(Vector2f initialPosition, std::vector<Vector2f> targets, std::vector<Obstacle> obstacles, Pose2f inversePose)
{   
    double score = 0.0;
    Vector2f bestTarget;
    
    for(Vector2f target : targets)
    {
        double computedScore = BehaviorUtilities::score(initialPosition,target,obstacles, inversePose);
        
        if(computedScore > score)
        {
            score = computedScore;
            bestTarget = target;
        }
    }
    return bestTarget;
}
