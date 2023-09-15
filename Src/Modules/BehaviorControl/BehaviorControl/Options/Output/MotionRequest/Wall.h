option(Wall, (float)(950.f) wallDistance)
{
    const float offsetPosition = 25.f;
    
    if (wallDistance < 950.f)
        wallDistance = 950.f;
        
    Vector2f goalPosition = Vector2f(0,0);
    if (theTeamBallModel.position.x() < theFieldDimensions.xPosOwnGoalArea)
    {
        goalPosition = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosCenterGoal);
    }
    else
    {
        if (theTeamBallModel.position.y() > theFieldDimensions.yPosCenterGoal)
        {
            goalPosition = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal);
        }
        else
        {
            goalPosition = Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal);
        }
    }

    Vector2f ballPosition = theTeamBallModel.position;

    Vector2f distanceFromBall = (goalPosition - ballPosition).normalized();
    Vector2f wallPosition = theRobotPose.inversePose*(theTeamBallModel.position + distanceFromBall*wallDistance);
    
    initial_state(walkToWallPosition)
    {
        transition
        {
            
            if(std::abs(wallPosition.x()) < offsetPosition &&
                std::abs(wallPosition.y()) < offsetPosition)
            {
                goto turnToBall;
            }
        }
        action
        {
            WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), wallPosition));
        }
    }

    state(turnToBall)
    {
        transition
        {
            if (std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
            {
                goto stand;
            }

            if(std::abs(wallPosition.x()) > offsetPosition ||
                std::abs(wallPosition.y()) > offsetPosition)
            {
                goto walkToWallPosition;
            }
        }
        action
        {
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
        }
    }

    state(stand)
    {
        transition
        {
            if (std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) > 5_deg)
            {
                goto turnToBall;
            }

            if(std::abs(wallPosition.x()) > offsetPosition ||
                std::abs(wallPosition.y()) > offsetPosition)
            {
                goto walkToWallPosition;
            }
        }
        action
        {
            Stand();
        }
    }
}