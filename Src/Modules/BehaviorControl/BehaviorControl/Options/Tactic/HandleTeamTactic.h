option(HandleTeamTactic)
{
  initial_state(ChooseRole)
  {
    transition
    {
      if(theRobotInfo.number == 1)
        goto PlayKeeper;
      else if(theBehaviorStatus.role == Role::supporter)
        goto PlaySupporter;
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
          theBehaviorStatus.role = Role::striker;
        else if(theRobotInfo.number == 3)
          theBehaviorStatus.role = Role::defender;
        else if(theRobotInfo.number == 4)
          theBehaviorStatus.role = Role::defender;
        else if(theRobotInfo.number == 5)
          theBehaviorStatus.role = Role::supporter;
        else if(theRobotInfo.number == 6)
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
  
  state(PlayDefender)
  {
    transition
    {
      if(action_aborted)
        if(theLibCodeRelease.nbOfStriker == 0 && theLibCodeRelease.nbOfSupporter == 0)
          goto PlayStriker;
    }
    action
    {
      theBehaviorStatus.role = Role::defender;
      LookForward();
      Defender();
    }
  }

  state(PlayStriker)
  {
    transition
    {
      if(action_aborted)
        if(!theLibCodeRelease.closerToTheBall)
          goto PlaySupporter;
    }
    action
    {
      theBehaviorStatus.role = Role::striker;
      LookForward();
      Striker();
    }
  }

  state(PlaySupporter)
  {
    transition
    {
      if(action_aborted) {
        if (theLibCodeRelease.nbOfDefender <= 1)
          goto PlayDefender;
        else if (theLibCodeRelease.closerToTheBall)
          goto PlayStriker;
        else{}
      }
    }
    action
    {
      theBehaviorStatus.role = Role::supporter;
      LookForward();
      Supporter();
    }
  }
}
