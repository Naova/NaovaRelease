option(WalkToSafeDistance)
{

    float offsetPosition = 25.f;

    initial_state(start)
    {
        transition
        {
            if ((theRobotPose.translation.x() > theTeamBallModel.position.x() &&
                theTeamBallModel.position.x() <= theFieldDimensions.xPosOpponentGroundline - 2000.f) ||
                theTeamBallModel.position.x() <= theFieldDimensions.xPosOwnGroundline + 2000.f)
            {
                goto walkTowardsOpponentSide;
            }
            else
            {
                goto walkTowardsOwnSide;
            }
        }
    }

    state(walkTowardsOpponentSide)
    {
        Vector2f position = theRobotPose.inversePose * Vector2f(theTeamBallModel.position.x() + 1000.f, theRobotPose.translation.y());
        transition
        {
            if((std::abs(position.x()) < offsetPosition &&
                std::abs(position.y()) < offsetPosition)||
                ((theRobotPose.inversePose*theTeamBallModel.position).norm() > theBehaviorParameters.safeDistance * 2) //*2 juste pour etre certain que si la position voulu est proche de la safe distance, on va bien a la bonne position
                )
            {
                goto finished;
            }
        }
        action
        {
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), position));
        }
    }

    state(walkTowardsOwnSide)
    {
        Vector2f position = theRobotPose.inversePose * Vector2f(theTeamBallModel.position.x() - 1000.f, theRobotPose.translation.y());
        transition
        {
            if((std::abs(position.x()) < offsetPosition &&
                std::abs(position.y()) < offsetPosition) ||
                ((theRobotPose.inversePose*theTeamBallModel.position).norm() > theBehaviorParameters.safeDistance * 2) //*2 juste pour etre certain que si la position voulu est proche de la safe distance, on va bien a la bonne position
                )
            {
                goto finished;
            }
        }
        action
        {
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), position));
        }
    }

    target_state(finished) {}
}