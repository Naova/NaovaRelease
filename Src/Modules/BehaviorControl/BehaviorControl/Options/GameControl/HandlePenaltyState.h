/** Handle penalty state (and the actions to be performed after leaving the penalty state).
 *   This option is one level higher than the main game state handling as penalties
 *   might occur in most game states. */
option(HandlePenaltyState)
{
  /** By default, the robot is not penalized and plays soccer according to the current game state.
  The chestbutton has to be pushed AND released to manually penalize a robot */
  initial_state(notPenalized)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
    }
    action
    {
	//	theArmMotionRequest = ArmKeyFrameRequest::back; Fix this line
		Naruto();
      HandleGameState();
    }
  }

  /** In case of any penalty, the robots stands still. */
  state(penalized)
  {
    transition
    {
      if(theRobotInfo.penalty == PENALTY_NONE)
        goto preUnpenalize;
    }
    action
    {
        theBehaviorStatus.role = Role::undefined;
      PlaySound("penalized.wav");
      theHeadControlMode = HeadControl::lookForward;
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }

  state(preUnpenalize)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      else if(theGameInfo.state == STATE_INITIAL || state_time > 5000)
        goto notPenalized;
    }
    action
    {
      PlaySound("notPenalized.wav");
      theHeadControlMode = HeadControl::lookForward;
      Stand();
      LookForward();
      WalkAtRelativeSpeed(Pose2f(0.f, 0.8, 0.f)); // Enter the field for the first time
    }
  }
}
