/* Move head from left to right
*/
int reps_temp;
int iter;

option(HeadSweep, (int)(1) reps, (float)(pi*0.4) speed)
{
  /** Starts moving to the right */
  initial_state(StartSearch)
  {
    transition
    {
      if (reps_temp == reps)
        goto moveSwitchRight;
    }
    action
    {
      reps_temp = reps;
      iter = 0;
    }
  }

  /* Checks if HeadSweep() was called again to start again*/
  target_state(finished)
  {
    transition
    {
      if (reps != reps_temp)
        goto StartSearch;
    }
  }

  /** Moves the head to the far right, then change the state to moveSwitchLeft */
  state(moveSwitchRight)
  {
     transition
    {
		  if(!theHeadMotionEngineOutput.moving && theHeadMotionEngineOutput.pan <= (-pi/4)) {
      iter++;
      goto moveSwitchLeft;
      }
    }
    action
    {
		SetHeadPanTilt((-pi/4)*1.2, 1.5,speed);
    }
  }

  /** Moves the head to the far left, if the ball has still not been found, we enter aborted_state */
  state(moveSwitchLeft)
  {
     transition
    {
      if(!theHeadMotionEngineOutput.moving && theHeadMotionEngineOutput.pan >= (pi/4)) {
        if (iter >= reps_temp)
          goto finished;
        else
          goto moveSwitchRight;
      }
    }
    action
    {
      SetHeadPanTilt((pi/4)*1.2, 1.5,speed);
    }
  }
}
