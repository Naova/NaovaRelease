option(HandleTeamTactic)
{
  common_transition
  {
    if(action_aborted)
    {
      switch (theRoleChanges.nextRole)
      {
        case Role::striker:
          goto PlayStriker;
          break;
        case Role::supporter:
          goto PlaySupporter;
          break;
        case Role::rightDefender:
          goto PlayRightDefender;
          break;
        case Role::leftDefender:
          goto PlayLeftDefender;
          break;
        case Role::defender:
          goto PlayDefender;
          break;
      }
    }
}

  initial_state(ChooseRole)
  {
    transition
    {
      if(theBehaviorStatus.role == Role::keeper)
        goto PlayKeeper;
      else if(theBehaviorStatus.role == Role::supporter)
        goto PlaySupporter;
      else if(theBehaviorStatus.role == Role::rightDefender)
        goto PlayRightDefender;
      else if(theBehaviorStatus.role == Role::leftDefender)
        goto PlayLeftDefender;
      else if(theBehaviorStatus.role == Role::defender)
        goto PlayDefender;
      else if(theBehaviorStatus.role == Role::striker)
        goto PlayStriker;
    }
    action
    {
      if(theBehaviorStatus.role == Role::undefined)
      {
        if(theRobotInfo.number == 1)
          theBehaviorStatus.role = Role::keeper;
        else if(theRobotInfo.number == 2)
          theBehaviorStatus.role = Role::rightDefender;
        else if(theRobotInfo.number == 3)
          theBehaviorStatus.role = Role::leftDefender;
        else if(theRobotInfo.number == 4)
          theBehaviorStatus.role = Role::supporter;
        else if(theRobotInfo.number == 5)
          theBehaviorStatus.role = Role::striker;
        else if(theRobotInfo.number == 6 || theRobotInfo.number == 8)
          theBehaviorStatus.role = Role::striker;
      }
      LookForward();
    }
  }

  state(PlayKeeper)
  {
    action
    {
      theBehaviorStatus.role = Role::keeper;
      LookForward();
      Keeper();
    }
  }
  
  state(PlayRightDefender)
  {
    action
    {

      theBehaviorStatus.role = Role::rightDefender;

      LookForward();
	    ArmInBack();
      RightDefender();
    }
  }

  state(PlayLeftDefender)
  {
    action
    {
      theBehaviorStatus.role = Role::leftDefender;

      LookForward();
	    ArmInBack();
      LeftDefender();
    }
  }

  state(PlayDefender)
  {
    action
    {

      theBehaviorStatus.role = Role::defender;

      LookForward();
	    ArmInBack();
      Defender();
    }
  }

  state(PlayStriker)
  {
    action
    {
      theBehaviorStatus.role = Role::striker;
      LookForward();
	    ArmInBack();
      Striker();
    }
  }

  state(PlaySupporter)
  {
    action
    {
      theBehaviorStatus.role = Role::supporter;
      LookForward();
	    ArmInBack();
      Supporter();
    }
  }
}
 