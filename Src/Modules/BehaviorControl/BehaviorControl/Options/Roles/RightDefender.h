option(RightDefender)
{    
    initial_state(start)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;
                
            if (state_time > 1000)
                goto turnToBall;
        }
        action
        {
            TrackBall();
            Stand();
        }
    }

    aborted_state(abortedState)
    {
        transition
        {
            if (theBehaviorStatus.role == Role::rightDefender)
                goto start;
        }
    }

    state(turnToBall)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;

            if(theTeamBallModel.isValid)
            {
                if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
                {
                    goto alignToBall;
                }
            }
            else
            {
                goto SearchBall;
            }
        }
        action
        {
            TrackBall();
            if(theTeamBallModel.isValid)
            {
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
            }
        }
    }

    state(SearchBall)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;

            if (theTeamBallModel.isValid)
            {
                goto turnToBall;
            }
        }
        action
        {
            SearchBall();
        }
    }

    state(alignToBall)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;

            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto turnToBall;
            }

            if(std::abs(theLibCodeRelease.defenderDesiredPos.x()) < 25 && std::abs(theLibCodeRelease.defenderDesiredPos.y()) < 25)
                goto stand;

            if(theLibCodeRelease.closerToTheBall || (theRobotPose.inversePose*theTeamBallModel.position).norm() <= theLibCodeRelease.defenderDistanceForKick)
                goto walkToBall;
            //si la balle zone danger
            //si la balle sort de la zone de position
        }
        action
        {
            TrackBall();
            WalkToTarget(Pose2f(90.f, 90.f, 90.f), Pose2f(theBallModel.estimate.position.angle(), theLibCodeRelease.defenderDesiredPos.x(), theLibCodeRelease.defenderDesiredPos.y()));
        }
    }

    state(walkToBall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto turnToBall;
            }

            if(!theLibCodeRelease.closerToTheBall && (theRobotPose.inversePose*theTeamBallModel.position).norm() > theLibCodeRelease.defenderDistanceForKick)
            {
                goto alignToBall;
            }

            if(action_done)
            {
                goto kick;
            }
        }
        action
        {
            Vector2f position = Vector2f(0,0);
            
            if(theTeamBallModel.position.y() >= 0)
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

    state(kick)
    {
        transition
        {
            if(state_time > 10 && action_done)
            {
                goto turnToBall;
            }
            
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto turnToBall;
            }

            if((theRobotPose.inversePose*theTeamBallModel.position).norm() >= 600.f)
            {
                goto walkToBall;
            }

            if(!theLibCodeRelease.closerToTheBall && (theRobotPose.inversePose*theTeamBallModel.position).norm() > theLibCodeRelease.defenderDistanceForKick)
            {
                goto alignToBall;
            }

            if(action_done){
                goto alignToBall;
            }
        }
        action
        {
            Vector2f position = Vector2f(0,0);
            
            if(theTeamBallModel.position.y() >= 0)
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

    state(stand)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;
            if(std::abs(theLibCodeRelease.defenderDesiredPos.x()) > 25 && std::abs(theLibCodeRelease.defenderDesiredPos.y()) > 25)
                goto alignToBall;
            if(theLibCodeRelease.closerToTheBall || (theRobotPose.inversePose*theTeamBallModel.position).norm() <= theLibCodeRelease.defenderDistanceForKick)
                goto walkToBall;

        }
        action
        {
            TrackBallSweep();
            Stand();
        }
    }
}