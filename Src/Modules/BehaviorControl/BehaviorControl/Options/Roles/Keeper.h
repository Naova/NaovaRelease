option(Keeper)
{
    common_transition
    {
        if(theLibCodeRelease.between(theBallModel.estimate.position.x(), 0, 1000.f)
        && theLibCodeRelease.between(theBallModel.estimate.position.y(), 150.f, 1000.f)
        && theLibCodeRelease.between(theBallModel.estimate.velocity.x(), -1000.f, -50.f))
            goto diveLeft;


        if(theLibCodeRelease.between(theBallModel.estimate.position.x(), 0, 1000.f)
        && theLibCodeRelease.between(theBallModel.estimate.position.y(), -1000.f, -150.f)
        && theLibCodeRelease.between(theBallModel.estimate.velocity.x(), -1000.f, -50.f))
            goto diveRight;
    }

    initial_state(start)
    {
        transition
        {
            if(state_time > 1000)
            {
                goto alignToBall;
            }
        }
        action
        {
            TrackBall();
            Stand();
        }
    }

    state(diveLeft)
    {
        transition
        {
        if(state_time > 10 && action_done)
            {
                goto start;
            }
        }
        action
        {
            SpecialAction(SpecialActionRequest::diveLeft);
        }
    }

    state(diveRight)
    {
        transition
        {
        if(state_time > 10 && action_done)
            {
                goto start;
            }
        }
        action
        {
            SpecialAction(SpecialActionRequest::diveRight);
        }
    }

    state(sumo)
    {
        action
        {
            SpecialAction(SpecialActionRequest::sumo);
            
        }
    }

    state(groundPunchLeft)
    {
        transition
        {
            if(state_time > 10 && action_done)
            {
                goto start;
            }
        }
        action
        {
            SpecialAction(SpecialActionRequest::groundPunchLeft);
        }
    }

    state(groundPunchRight)
    {
        transition
        {
            if(state_time > 10 && action_done)
            {
                goto start;
            }
        }
        action
        {
            SpecialAction(SpecialActionRequest::groundPunchRight);
        }
    }

    state(alignToBall)
    {
        Vector2f centerGoal = Vector2f(theFieldDimensions.xPosOwnGroundline, theTeamBallModel.position.y()*0.02f);
        Vector2f vector = theTeamBallModel.position - centerGoal;
        Vector2f vectorNormalized = vector.normalized();
        Vector2f position = (centerGoal + (vectorNormalized * (std::abs(theFieldDimensions.yPosLeftGoal) * 0.7f)));

        Vector2f movementVector = theRobotPose.inversePose * position;

        transition
        {
            if (theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto walkToStandardPosition;
            }

            if(theBallModel.estimate.position.norm() < theLibCodeRelease.keeperDistanceForKick || theLibCodeRelease.closerToTheBall)
                goto walkToBall;

            if (std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg && movementVector.norm() < 30)
            {
                goto standby;
            }
        }
        action
        {
            TrackBall();
            WalkToTarget(Pose2f(45.f, 45.f, 45.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), movementVector));
        }
    }

    state(walkToBall)
    {
        transition
        {
            if (theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto walkToStandardPosition;
            }

            if(!(theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() < theLibCodeRelease.keeperDistanceForKick))
            {
                if (theLibCodeRelease.robotIsInPenaltyArea)
                {
                    goto alignToBall;
                }
                else
                {
                    goto walkToStandardPosition;
                }
            }

            if(action_done)
            {
                goto alignBehindBall;
            }
        }
        action
        {
            Vector2f position = Vector2f(0,0);
        
            if(theTeamBallModel.position.y() > 0)
            {
                position = Vector2f(theFieldDimensions.xPosOpponentGroundline,theFieldDimensions.centerCircleRadius);
            }
            else
            {
                position = Vector2f(theFieldDimensions.xPosOpponentGroundline,-theFieldDimensions.centerCircleRadius);
            }

            TrackBallModel();
            WalkToBall(position);
        }
    }

    state(alignBehindBall)
    {
        transition
        {
            if(!(theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() < theLibCodeRelease.keeperDistanceForKick))
            {
                if (theLibCodeRelease.robotIsInPenaltyArea)
                {
                    goto alignToBall;
                }
                else
                {
                    goto walkToStandardPosition;
                }
            }

            if(theBallModel.estimate.position.norm() > 600.f)
            {
                goto walkToBall;
            }

            if(action_done)
            {
                goto walkToStandardPosition;
            }

        }
        action
        {
            TrackBall();

            for(auto const& tm : theTeamData.teammates)
            {

                float furthestSupporterXPos = theFieldDimensions.xPosOwnPenaltyMark / 2.f;

                if(tm.theBehaviorStatus.role == Role::supporter 
                    && tm.theRobotPose.translation.x() < furthestSupporterXPos)
                {
                    Pass(tm, 1250.f);
                    return;
                }
            }

            Vector2f position = Vector2f(0,0);
        
            if(theTeamBallModel.position.y() > 0)
            {
                position = Vector2f(theFieldDimensions.xPosOpponentGroundline,theFieldDimensions.centerCircleRadius);
            }
            else
            {
                position = Vector2f(theFieldDimensions.xPosOpponentGroundline,-theFieldDimensions.centerCircleRadius);
            }
            KickWithPosition(position);
        }
    }

    state(walkToStandardPosition)
    {
        Vector2f standardPosition = theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline + theLibCodeRelease.safeDistance, theFieldDimensions.yPosCenterGoal);
        
        transition
        {
            if (std::abs(standardPosition.x()) < 25 && std::abs(standardPosition.y()) < 25)
            {
                goto turnToCenter;
            }

            if (theTeamBallModel.isValid && theLibCodeRelease.robotIsInPenaltyArea)
            {
                goto alignToBall;
            }
        }
        action
        {
            TrackBall();
            if (theTeamBallModel.isValid)
            {
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), standardPosition));
            }
            else
            {
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose*Vector2f(0,0)).angle(), standardPosition));
            }
        }
    }

    state(turnToCenter)
    {
        Vector2f standardPosition = theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline + theLibCodeRelease.safeDistance, theFieldDimensions.yPosCenterGoal);

        transition
        {
            if (std::abs(standardPosition.x()) >= 25 && std::abs(standardPosition.y()) >= 25)
            {
                goto walkToStandardPosition;
            }

            if(std::abs((theRobotPose.inversePose * Vector2f(0,0)).angle()) < 5_deg)
                goto standby;

            if (theTeamBallModel.isValid)
            {
                goto alignToBall;
            }
        }
        action
        {
            TrackBall();
            WalkToTarget(Pose2f(4.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(0,0)).angle(), 0.f, 0.f));
        }
    }

    state(standby)
    {
        Vector2f standardPosition = theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline + theLibCodeRelease.safeDistance, theFieldDimensions.yPosCenterGoal);
        
        transition
        {

            if (theTeamBallModel.isValid)
            {
                if (theLibCodeRelease.robotIsInPenaltyArea && std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) >= 5_deg)
                {
                    goto alignToBall;
                }
                
                if (!theLibCodeRelease.robotIsInPenaltyArea)
                {
                    goto walkToStandardPosition;
                }
            }

            if (!theTeamBallModel.isValid && std::abs(standardPosition.x()) >= 25 && std::abs(standardPosition.y()) >= 25)
            {
                goto walkToStandardPosition;
            }
        }

        action
        {
            TrackBallSweep();
            Stand();
        }
    }
}