/**
 * @file DribblePathProvider.cpp
 *
 * This file implements a module that determines optimal dribbling paths around obstacles.
 *
 * @author Aissa Bouaraguia
 */

#include "DribblePathProvider.h"
#include <iostream>



MAKE_MODULE(DribblePathProvider, behaviorControl);



void DribblePathProvider::update(DribblePath& theDribblePath)
{
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:direction", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:step", "drawingOnField");
    

    if(!theTeamBehaviorStatus.role.playsTheBall()){
        return;
    }

    std::vector<Vector2f> stepPositions;
    std::vector<Vector2f> stepDirections;

    Angle lastDribbleAngle = 0_deg;
    PotentialValue potentialValue;
    Vector2f position = theFieldBall.endPositionOnField;

    const Vector2f startPosition = position - position.normalized(0.001f); // To ensure we are not on the field line
    position += Vector2f::polar(100.f, lastDribbleAngle);

    for(int i = 0; i < iterationSteps; i++)
    {
        potentialValue = theFieldRating.potentialFieldOnly(position.x(), position.y(), true);
        theFieldRating.getObstaclePotential(potentialValue, position.x(), position.y(), true);
        theFieldRating.potentialWithRobotFacingDirection(potentialValue, position.x(), position.y(), true);
        const float directionNorm = potentialValue.direction.norm();
        if(directionNorm == 0.f)
            break;

        Vector2f direction = potentialValue.direction / directionNorm;
        direction *= stepLength;

        // Save positions and directions
        stepPositions.push_back(position);
        stepDirections.push_back(direction);

        COMPLEX_DRAWING("skill:DribbleToGoal:step")
        {
        if(i % searchStepDrawModulo == 0 && i != 0) // skip first
            ARROW("skill:DribbleToGoal:step", position.x(), position.y(), position.x() + searchStepDrawScale * direction.x(), position.y() + searchStepDrawScale * direction.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::black);
        }


        position += direction;
        if(position.x() > theFieldDimensions.xPosOpponentGroundLine && position.y() < theFieldDimensions.yPosLeftGoal && position.y() > theFieldDimensions.yPosRightGoal)
        break;
    }

    Angle dribbleAngleInField = (position - startPosition).angle();

    COMPLEX_DRAWING("skill:DribbleToGoal:direction")
    {
        const Vector2f pos2 = theFieldBall.endPositionOnField + Vector2f::polar(300.f, dribbleAngleInField);
        ARROW("skill:DribbleToGoal:direction", theFieldBall.endPositionOnField.x(), theFieldBall.endPositionOnField.y(), pos2.x(), pos2.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::cyan);
    }

    const Angle dribbleAngle = Angle::normalize(dribbleAngleInField - theRobotPose.rotation);
    lastDribbleAngle = dribbleAngle;




    float closestObstacleDistance = 20000.0f; //Set a max distance
    for(const Obstacle& obstacle : theObstacleModel.obstacles) //Find the smallest distance among the obstacles
    {
      if (obstacle.isOpponent() && obstacle.center.x() > 0 )
      {
        float distanceObstacle = obstacle.center.norm();
        if(distanceObstacle < closestObstacleDistance)
        {
          closestObstacleDistance = distanceObstacle;
        }
      }
    }
    float dribbleSpeed = minimumDribbleSpeed + (maximumDribbleSpeed - minimumDribbleSpeed) * ((closestObstacleDistance - minimumDistance)/maximumDistance); //Set the dribble speed depending on the distance of the closest opponent
    
    if(dribbleSpeed < minimumDribbleSpeed)
        dribbleSpeed = minimumDribbleSpeed;
        
    if(dribbleSpeed > maximumDribbleSpeed)
        dribbleSpeed = maximumDribbleSpeed;
    

    theDribblePath.dribbleSpeed = dribbleSpeed;
    theDribblePath.dribbleAngle = dribbleAngle;
    theDribblePath.stepPositions = stepPositions;
    theDribblePath.stepDirections = stepDirections;

}

