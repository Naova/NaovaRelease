option(Supporter)
{
  common_transition
  {
    if(theLibCodeRelease.nbOfDefender <= 1)
    {
        theLibCodeRelease.setMissingTeammate(true);
      if(theLibCodeRelease.timeSinceMissingTeammate > theLibCodeRelease.waitToChangeRole + theRobotInfo.number*500)
      {
          theLibCodeRelease.setMissingTeammate(false);
            goto abortedState;
      }
    }
    else
    {
        theLibCodeRelease.setMissingTeammate(false);
    }
    if(theLibCodeRelease.closerToTheBall)
    {
      if(state_time > theRobotInfo.number*500)
      {
        goto abortedState;
      }
    }
  }

  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
        goto turnToBall;
    }
    action
    {
      LookForward();
      Stand();
    }
  }

  aborted_state(abortedState){}

  state(turnToBall)
  {
      transition
      {
          if(theTeamBallModel.isValid)
          {
              if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
              {
                  goto walkToSupportPos;
              }
          }
          if(state_time > 5000)
          {
              goto walkToLastBall;
          }
      }
      action
      {
          LookForward();
          if(theTeamBallModel.isValid)
          {
              WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
          }
          else
          {
              //WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
          }
      }
  }

  state(walkToLastBall)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen < theBehaviorParameters.ballNotSeenTimeOut)
          {
              goto turnToBall;
          }
          if(theBallModel.estimate.position.norm() < 500.f)
          {
              goto walkToSupportPos;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), theRobotPose.inversePose * theTeamBallModel.position));
      }
  }

  state(walkToSupportPos)
  {
    transition
    {
        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        {
            goto turnToBall;
        }
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToOppGoal, theLibCodeRelease.desiredPos));
    }
  }
}
