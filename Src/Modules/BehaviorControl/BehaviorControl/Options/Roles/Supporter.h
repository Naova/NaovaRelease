option(Supporter)
{
  initial_state(start)
  {
    transition
    {
        if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
            goto abortedState;
        if(state_time > 1000)
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
        if (theBehaviorStatus.role == Role::supporter)
            goto start;
    }
  }

  state(turnToBall)
  {
    transition
    {
        if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
            goto abortedState;
        if (theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() <= theLibCodeRelease.supporterDistanceForKick)
            goto walkToBall;
        if(theTeamBallModel.isValid)
            {
                if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
                {
                    goto walkToSupportPos;
                }
            }
        else if (state_time > 5000)
        {
            goto walkToLastBall;
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

  state(walkToLastBall)
  {
    transition
    {
        if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
            goto abortedState;
        if (theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() <= theLibCodeRelease.supporterDistanceForKick)
            goto walkToBall;

        if(theLibCodeRelease.timeSinceBallWasSeen < theBehaviorParameters.ballNotSeenTimeOut)
        {
            goto turnToBall;
        }
        if(theBallModel.estimate.position.norm() > 600.f)
        {
            goto walkToSupportPos;
        }
    }
    action
    {
        TrackBall();
        WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle(), theBallModel.estimate.position), 1);
    }
  }

  state(walkToSupportPos)
  {
    transition
    {
        if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
            goto abortedState;
        if (theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() <= theLibCodeRelease.supporterDistanceForKick)
            goto walkToBall;
        
        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        {
            goto turnToBall;
        }
    }
    action
    {
	    TrackBall();
        if (theTeamBallModel.position.x() > 0.f)
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToOppGoal, theLibCodeRelease.desiredPos));
        else
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), theLibCodeRelease.desiredPos));
    }
  }


  state(walkToBall)
  {
    transition
    {
        if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
            goto abortedState;
        if(!(theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() <= theLibCodeRelease.supporterDistanceForKick))
        {
            goto walkToSupportPos;
        }
        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        {
            goto turnToBall;
        }
        if(action_done)
        {
            goto kick;
        }
    }
    action
    {
        TrackBallModel();
        WalkToBall(theLibCodeRelease.supporterDesiredKickPos);
    }
  }

  state(kick)
  {
    transition
    {
        if(!(theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() <= theLibCodeRelease.supporterDistanceForKick))
        {
            goto walkToSupportPos;
        }
        if(state_time > 10 && action_done)
        {
            goto walkToSupportPos;
        }
        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        {
            goto turnToBall;
        }
        if(theBallModel.estimate.position.norm() > 600.f)
        {
            goto walkToBall;
        }
    }
    action
    {
        TrackBall();
        KickWithPosition(theLibCodeRelease.supporterDesiredKickPos);
    }
  }
}
