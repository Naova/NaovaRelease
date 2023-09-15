/** Handle penalty state (and the actions to be performed after leaving the penalty state).
 *   This option is one level higher than the main game state handling as penalties
 *   might occur in most game states. */
option(HandlePenaltyState)
{
  float offsetPosition = 25.f;

  /** By default, the robot is not penalized and plays soccer according to the current game state.
  The chestbutton has to be pushed AND released to manually penalize a robot */
  initial_state(notPenalized)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE && theRobotInfo.mode != RobotInfo::unstiff)
        goto penalized;
    }
    action
    {
	//	theArmMotionRequest = ArmKeyFrameRequest::back; Fix this line
		
      HandleGameState();
    }
  }

  /** In case of any penalty, the robots stands still. */
  state(penalized)
  {
    transition
    {
      if(theRobotInfo.penalty == PENALTY_NONE && theRobotInfo.mode != RobotInfo::unstiff)
        goto sweep;
    }
    action
    {
      if (theTeamData.canSendMessages)
      {
        theBehaviorStatus.role = Role::undefined;
      }
      theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
      theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;
      PlaySound("penalized.wav");
      SetHeadPanTilt(0, 0);
      theHeadControlMode = HeadControl::lookForward;
      SpecialAction(SpecialActionRequest::standHigh);
    }
  }

  state(preUnpenalizeKeeper)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      if(theGameInfo.state != STATE_PLAYING)
        goto notPenalized;
      else if (theBehaviorStatus.role == Role::keeper && theTeamBallModel.isValid)
        goto notPenalized;
      else if (action_done)
        goto notPenalized;
    }
    action
    {
      TrackBallSweep();

      MovePlayerToStartPosition();
    }
  }

  state(preUnpenalizeDefender)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      if(theGameInfo.state != STATE_PLAYING)
        goto notPenalized;
      else if ((theBehaviorStatus.role == Role::rightDefender || theBehaviorStatus.role == Role::leftDefender) && theTeamBallModel.isValid)
        goto notPenalized;
      else if (theGameInfo.kickingTeam != Global::getSettings().teamNumber)
      {
        if (theBehaviorStatus.role == Role::leftDefender && (std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoalArea / 2.f)).x()) < offsetPosition &&
          std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoalArea / 2.f)).y()) < offsetPosition))
          goto turnToCenter;
        else if (theBehaviorStatus.role == Role::rightDefender && (std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoalArea / 2.f)).x()) < offsetPosition &&
          std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoalArea / 2.f)).y()) < offsetPosition))
          goto turnToCenter;
      }
      else if (action_done)
        goto notPenalized;
    }
    action
    {
      TrackBallSweep();

      if (theGameInfo.kickingTeam == Global::getSettings().teamNumber)
        {
          MovePlayerToStartPosition();
        }
        else
        {
          if(theBehaviorStatus.role == Role::leftDefender)
            {
                Vector2f leftDefenderPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoalArea / 2.f));

                WalkToTarget(Pose2f(5.f, 100.f, 100.f),Pose2f(leftDefenderPosition.angle(), leftDefenderPosition), 1);
            }
            else
            {
                Vector2f rightDefenderPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoalArea / 2.f));

                WalkToTarget(Pose2f(5.f, 100.f, 100.f),Pose2f(rightDefenderPosition.angle(), rightDefenderPosition), 1);
            }
        }
    }
  }

  state(preUnpenalizeSupporter)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      if(theGameInfo.state != STATE_PLAYING)
        goto notPenalized;
      else if (theBehaviorStatus.role == Role::supporter && theTeamBallModel.isValid)
        goto notPenalized;
      else if (theBehaviorStatus.role == Role::supporter && (std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline / 3, theFieldDimensions.yPosCenterGoal)).x()) < offsetPosition &&
        std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline / 3, theFieldDimensions.yPosCenterGoal)).y()) < offsetPosition))
        goto turnToCenter;
    }
    action
    {
      TrackBallSweep();

      Vector2f supporterPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline / 3, theFieldDimensions.yPosCenterGoal));

      WalkToTarget(Pose2f(5.f, 100.f, 100.f), Pose2f(supporterPosition.angle(), supporterPosition), 1);
    }
  }

  state(preUnpenalizeStriker)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      if(theGameInfo.state != STATE_PLAYING)
        goto notPenalized;
      else if (theBehaviorStatus.role == Role::striker && theTeamBallModel.isValid)
        goto notPenalized;
      else if (theBehaviorStatus.role == Role::striker && (std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosHalfWayLine + (theFieldDimensions.centerCircleRadius / 2), theFieldDimensions.yPosCenterGoal)).x()) < offsetPosition &&
        std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosHalfWayLine + (theFieldDimensions.centerCircleRadius / 2), theFieldDimensions.yPosCenterGoal)).y()) < offsetPosition))
        goto turnToCenter;
    }
    action
    {
      TrackBallSweep();

      Vector2f strikerPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosHalfWayLine + (theFieldDimensions.centerCircleRadius / 2), theFieldDimensions.yPosCenterGoal));

      WalkToTarget(Pose2f(5.f, 100.f, 100.f), Pose2f(strikerPosition.angle(), strikerPosition), 1);
    }
  }

  state(sweep)
  {
    transition
    {
      if(theRobotInfo.penalty != PENALTY_NONE)
        goto penalized;
      if(theGameInfo.state != STATE_PLAYING)
        goto notPenalized;
      else if (action_done)
      {
        if (theBehaviorStatus.role == Role::keeper)
          goto preUnpenalizeKeeper;
        else if (theBehaviorStatus.role == Role::rightDefender || theBehaviorStatus.role == Role::leftDefender)
          goto preUnpenalizeDefender;
        else if (theBehaviorStatus.role == Role::supporter)
          goto preUnpenalizeSupporter;
        else if (theBehaviorStatus.role == Role::striker)
          goto preUnpenalizeStriker;
      }
    }
    action
    {
      if (theTeamData.canSendMessages)
      {
        theBehaviorStatus.role = theLibCodeRelease.desiredRole;
      }
      else if (!theTeamData.canSendMessages && theBehaviorStatus.role == Role::undefined)
      {
        theBehaviorStatus.role = Role::supporter;
      }
      PlaySound("notPenalized.wav");
      HeadSweep();
    }
  }

  state(stand)
  {
    transition
    {
      goto notPenalized;
    }
    action
    {
        theHeadControlMode = HeadControl::lookForward;
        Stand();
    }
  }

  state(turnToCenter)
  {
    transition
    { 
      if(theGameInfo.state != STATE_PLAYING)
        goto notPenalized;
      if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle()) < 6_deg)
        goto stand;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(4.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle(), 0.f, 0.f), 1);
    }
  }
}
