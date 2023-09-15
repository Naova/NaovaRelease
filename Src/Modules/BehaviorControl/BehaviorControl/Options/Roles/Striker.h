/** A test striker option without common decision */
option(Striker)
{
  Pose2f position =(theRobotPose.inversePose * Vector2f(theFieldDimensions.centerCircleRadius, 0));
  
  initial_state(start)
  {
    transition
    {
      if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
        goto abortedState;

      if(state_time > 1000)
        goto turnToBall;
      
      if(theLibCodeRelease.nbOfPlayers - theLibCodeRelease.nbOfKeeper >= 1 
        && theTeamBallModel.position.x() < 0
        && !theLibCodeRelease.between(position.translation.x(), -100.f, 100.f)
        && !theLibCodeRelease.between(position.translation.y(), -100.f, 100.f))
      {      
        goto offensivePosition;
      }
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
        if (theBehaviorStatus.role == Role::striker)
            goto start;
    }
}

  state(turnToBall)
  {
    transition
    {
      if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
        goto abortedState;
      
      if(theLibCodeRelease.nbOfPlayers - theLibCodeRelease.nbOfKeeper >= 1 
        && theTeamBallModel.position.x() < 0
        && !theLibCodeRelease.between(position.translation.x(), -100.f, 100.f)
        && !theLibCodeRelease.between(position.translation.y(), -100.f, 100.f))
      {      
        goto offensivePosition;
      }  

      if(theTeamBallModel.isValid)
      {
        if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
        {
          goto walkToBall;
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
      if(theTeamBallModel.isValid)
      {
        TrackBall();
        WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
      }
      else
      {
        SearchBall();
      }
    }
  }

	state(SearchBall)
	{
		transition
		{
      if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
        goto abortedState;

      if(theLibCodeRelease.nbOfPlayers - theLibCodeRelease.nbOfKeeper >= 1 
        && theTeamBallModel.position.x() < 0
        && !theLibCodeRelease.between(position.translation.x(), -100.f, 100.f)
        && !theLibCodeRelease.between(position.translation.y(), -100.f, 100.f))
      {      
        goto offensivePosition;
      }

      if (theTeamBallModel.isValid)
      {
        if(theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() < theLibCodeRelease.strikerDistanceForKick)
        {
          goto walkToBall;
        }
			  if(theTeamBallModel.position.x() > 0)
        {
          goto turnToBall;
        }
        if(theLibCodeRelease.nbOfPlayers - theLibCodeRelease.nbOfKeeper >= 1 
          && theTeamBallModel.position.x() < 0
          && theLibCodeRelease.between(position.translation.x(), -100.f, 100.f)
          && theLibCodeRelease.between(position.translation.y(), -100.f, 100.f))
        {      
          goto standToPosition;
        }
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

      if(theLibCodeRelease.nbOfPlayers - theLibCodeRelease.nbOfKeeper >= 1 
        && theTeamBallModel.position.x() < 0
        && !theLibCodeRelease.between(position.translation.x(), -100.f, 100.f)
        && !theLibCodeRelease.between(position.translation.y(), -100.f, 100.f))
      {      
        goto offensivePosition;
      }

      if(theLibCodeRelease.timeSinceBallWasSeen < theBehaviorParameters.ballNotSeenTimeOut)
      {
        goto turnToBall;
      }
      if(theBallModel.estimate.position.norm() < 600.f)
      {
        goto alignBehindBall;
      }
    }
    action
    {
      TrackBall();
      WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theBallModel.estimate.position.angle(), theBallModel.estimate.position), 1);
    }
  }

  state(walkToBall)
  {
    transition
    {
      if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
        goto abortedState;
      if(theLibCodeRelease.nbOfPlayers - theLibCodeRelease.nbOfKeeper >= 1 && theTeamBallModel.position.x() < 0
        && !theLibCodeRelease.between(position.translation.x(), -100.f, 100.f)
        && !theLibCodeRelease.between(position.translation.y(), -100.f, 100.f))
      {      
        goto offensivePosition;
      }
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
      {
        goto turnToBall;
      }
      if(action_done)
      {
        goto alignBehindBall;
      }
    }
    action
    {
      TrackBallModel();
      WalkToBall(theLibCodeRelease.desiredKickPos);
    }
  }

  state(alignBehindBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
      {
        goto turnToBall;
      }
      if(theBallModel.estimate.position.norm() > 600.f)
      {
        goto walkToBall;
      }
      
      if(action_done)
      {
        goto walkToBall;
      }
    }
    action
    {
      KickWithPosition(theLibCodeRelease.desiredKickPos);
    }
  }

  state(offensivePosition)
  {
    transition
    {
      if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
        goto abortedState;

      if(theTeamBallModel.position.x() > 0 || theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() < theLibCodeRelease.strikerDistanceForKick)
      {
        goto walkToBall;
      }

      Vector2f angleVector = (theRobotPose.inversePose * Vector2f(0, 0));

      if(theLibCodeRelease.between(position.translation.x(), -100.f, 100.f)
          && theLibCodeRelease.between(position.translation.y(), -100.f, 100.f)
          &&std::abs(angleVector.angle()) < 5_deg){
        goto SearchBall;
      }

    }
    action
    {    
      Vector2f offensivePosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.centerCircleRadius, 0));

      TrackBall();
      WalkToTarget(Pose2f(40.f, 60.f, 60.f), Pose2f(offensivePosition.angle(), offensivePosition), 1);
    }
  }

  state(standToPosition)
  {
    transition
    {
      if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
        goto abortedState;

      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut) 
      {
        goto turnToBall;
      }

      if(theTeamBallModel.position.x() > 0 || theLibCodeRelease.closerToTheBall || theBallModel.estimate.position.norm() < theLibCodeRelease.strikerDistanceForKick)
      {
        goto walkToBall;
      }
    }
    action
    {    
      TrackBallSweep();
      Stand();
    }
  }
}
