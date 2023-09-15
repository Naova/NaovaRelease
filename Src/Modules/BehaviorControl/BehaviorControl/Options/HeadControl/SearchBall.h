/* Search for the ball if the teamball is not valid
*  We start by moving the head to the far right, then the far left (We only do this once)
*  If the ball gets found, we enter target_state
*  If the ball doesn't get found, we enter aborted_state
*/
int direction = 1;

option(SearchBall)
{
  common_transition
  {
    if(theTeamBallModel.isValid)
        goto ballFound;
  }

  /** Starts moving to the right */
  initial_state(StartSearch)
  {
    transition
    {
      if(!theTeamBallModel.isValid)
        goto moveSwitchRight;
    }
    action
    {
      if(theBallModel.lastPerception.angle() < 0)
      {
        direction = -1;
      }
      else
      {
        direction = 1;
      }
    }
  }

  target_state(ballFound)
  {
    action
    {
      // When we are in this state, it means the ball has been found
    }
  }

  /** Moves the head to the far right, then change the state to moveSwitchLeft */
  state(moveSwitchRight)
  {
     transition
    {
      if (!theHeadMotionEngineOutput.moving && theHeadMotionEngineOutput.pan >= (pi / 4))
      {
        goto ballNotFound;
      }
      if(!theHeadMotionEngineOutput.moving && theHeadMotionEngineOutput.pan <= (-pi/4))
      {
        goto moveSwitchLeft;
      }
    }
    action
    {
		 SetHeadPanTilt((-pi/4)*1.2, 1.5,pi*0.4);
    }
  }

  /** Moves the head to the far left, if the ball has still not been found, we enter aborted_state */
  state(moveSwitchLeft)
  {
     transition
    {
      if(!theHeadMotionEngineOutput.moving && theHeadMotionEngineOutput.pan >= (pi/4))
      {
        goto ballNotFound;
      }
    }
    action
    {
      SetHeadPanTilt((pi/4)*1.2, 1.5,pi*0.4);
    }
  }

  state(SearchBallTriangle)
  {
	  transition
	  {
		  if (theTeamBallModel.isValid)
		  {
			  goto ballFound;
		  }
	  }
		  action
	  {
		  SearchBallTriangle();
	  }
  }

  // If we are here, the ball hasn't been found, so the position of the NAO should be changed
  aborted_state(ballNotFound)
  {
    transition
    {
      if(action_done)
        goto moveSwitchRight;
    }
    action
    {
      SetHeadPanTilt(0, 0);
      
      if(theBehaviorStatus.role != Role::keeper)
      {
        TurnInPlace(direction * 0.7f, 120_deg);
      }
    }
  }
}
