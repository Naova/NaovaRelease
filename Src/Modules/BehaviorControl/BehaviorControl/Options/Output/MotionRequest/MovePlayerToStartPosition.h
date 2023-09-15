/*
  Move the player to his starting position.
*/
option(MovePlayerToStartPosition)
{
  Vector2f keeperFirstPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGoalArea + theLibCodeRelease.safeDistance, theFieldDimensions.yPosLeftGoalArea));
  Vector2f keeperFinalPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline + theLibCodeRelease.safeDistance,0));
  Vector2f offensiveLeftDefenderPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea - theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoalArea / 2.f));
  Vector2f defensiveLeftDefenderPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGoalArea + (theFieldDimensions.xPosHalfWayLine - theFieldDimensions.xPosOwnGoalArea) * 0.5f + theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoalArea * 0.9f));
  Vector2f offensiveRightDefenderPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea - theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoalArea / 2.f));
  Vector2f defensiveRightDefenderPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark + theLibCodeRelease.safeDistance,theFieldDimensions.yPosCenterGoal));
  Vector2f offensiveSupporterPosition = (theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 500, theFieldDimensions.yPosRightGoal));
  Vector2f defensiveSupporterPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGoalArea + (theFieldDimensions.xPosHalfWayLine - theFieldDimensions.xPosOwnGoalArea) * 0.5f + theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoalArea * 0.9f));
  Vector2f offensiveStrikerPosition = (theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 200, 0.f));
  Vector2f defensiveStrikerPosition = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosHalfWayLine + (theFieldDimensions.xPosHalfWayLine - theFieldDimensions.centerCircleRadius) - theLibCodeRelease.safeDistance * 2.f, 0));

  float offsetPosition = 25.f;

  initial_state(setRequest)
  {
    transition
    {
      if(theBehaviorStatus.role == Role::keeper)
        goto moveKeeperToFirstStartPosition;
      else if(theBehaviorStatus.role == Role::rightDefender || theBehaviorStatus.role == Role::leftDefender)
        goto moveDefenderToStartPosition;
      else if(theBehaviorStatus.role == Role::supporter)
        goto moveSupporterToStartPosition;
      else if(theBehaviorStatus.role == Role::striker)
        goto moveStrikerToStartPosition;
      
    }
  }

  state(moveKeeperToFirstStartPosition)
  {
    transition
    {
      if(std::abs(keeperFirstPosition.x()) < offsetPosition &&
        std::abs(keeperFirstPosition.y()) < offsetPosition)
        goto moveKeeperToFinalStartPosition;
      if(Geometry::isPointInsideRectangle2(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoalArea),
                                                Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea),
                                                theRobotPose.translation))
                                                goto moveKeeperToFinalStartPosition;
    }
    action
    {
      WalkToTarget(Pose2f(5.f, 70.f, 70.f),Pose2f(keeperFirstPosition.angle(), keeperFirstPosition), 1);
    }
  }

  state(moveKeeperToFinalStartPosition)
  {
    transition
    {
      if(std::abs(keeperFinalPosition.x()) < offsetPosition &&
        std::abs(keeperFinalPosition.y()) < offsetPosition)
        goto turnToCenter;
    }
    action
    {
      WalkToTarget(Pose2f(5.f, 70.f, 70.f),Pose2f(keeperFinalPosition.angle(), keeperFinalPosition), 1);
    }
  }


  state(moveDefenderToStartPosition)
  {
      transition
      {
        if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
        {
          if(theBehaviorStatus.role == Role::leftDefender){
              if(std::abs(offensiveLeftDefenderPosition.x()) < offsetPosition &&
                 std::abs(offensiveLeftDefenderPosition.y()) < offsetPosition)
                  goto turnToCenter;
          }
          else
          {
              if(std::abs(offensiveRightDefenderPosition.x()) < offsetPosition &&
                 std::abs(offensiveRightDefenderPosition.y()) < offsetPosition)
                  goto turnToCenter;
          }
        }
        else
        {
          if(theBehaviorStatus.role == Role::leftDefender){
              if(std::abs(defensiveLeftDefenderPosition.x()) < offsetPosition &&
                 std::abs(defensiveLeftDefenderPosition.y()) < offsetPosition)
                  goto turnToCenter;
          }
          else
          {
              if(std::abs(defensiveRightDefenderPosition.x()) < offsetPosition &&
                 std::abs(defensiveRightDefenderPosition.y()) < offsetPosition)
                  goto turnToCenter;
          }
        }
      }
      action
      {
        if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
        {
            if(theBehaviorStatus.role == Role::leftDefender)
            {
                WalkToTarget(Pose2f(5.f, 70.f, 70.f),Pose2f(offensiveLeftDefenderPosition.angle(), offensiveLeftDefenderPosition), 1);
            }
            else
            {
                WalkToTarget(Pose2f(5.f, 70.f, 70.f),Pose2f(offensiveRightDefenderPosition.angle(), offensiveRightDefenderPosition), 1);
            }
        }
        else
        {
            if(theBehaviorStatus.role == Role::leftDefender)
            {
                WalkToTarget(Pose2f(5.f, 70.f, 70.f),Pose2f(defensiveLeftDefenderPosition.angle(), defensiveLeftDefenderPosition), 1);
            }
            else
            {
                WalkToTarget(Pose2f(5.f, 70.f, 70.f),Pose2f(defensiveRightDefenderPosition.angle(), defensiveRightDefenderPosition), 1);
            }
        }
      }
  }

  state(moveSupporterToStartPosition)
  {
      transition
      {
        if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
        {
          if((std::abs(offensiveSupporterPosition.x()) < offsetPosition) &&
              (std::abs(offensiveSupporterPosition.y()) < offsetPosition))
          {
              goto turnToCenter;
          }
        }
        else
        {
            if(std::abs(defensiveSupporterPosition.x()) < offsetPosition &&
                 std::abs(defensiveSupporterPosition.y()) < offsetPosition)
                  goto turnToCenter;
        }
      }
      action
      {
        if(theGameInfo.kickingTeam == Global::getSettings().teamNumber) 
        {
            WalkToTarget(Pose2f(5.f, 70.f, 70.f), Pose2f(offensiveSupporterPosition.angle(), offensiveSupporterPosition), 1);
        }
        else
        {
            WalkToTarget(Pose2f(5.f, 70.f, 70.f), Pose2f(defensiveSupporterPosition.angle(), defensiveSupporterPosition), 1);
        }
      }
  }

  state(moveStrikerToStartPosition)
  {
      transition
      {
          if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
          {
              if(std::abs(offensiveStrikerPosition.x()) < offsetPosition &&
                  std::abs(offensiveStrikerPosition.y()) < offsetPosition)
                  goto turnToCenter;
          }
          else
          {
              if(std::abs(defensiveStrikerPosition.x()) < offsetPosition &&
                  std::abs(defensiveStrikerPosition.y()) < offsetPosition)
                  goto turnToCenter;
          }


      }
      action
      {
          if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
          {
              WalkToTarget(Pose2f(5.f, 70.f, 70.f), Pose2f(offensiveStrikerPosition.angle(), offensiveStrikerPosition), 1);
          }
          else
          {
              WalkToTarget(Pose2f(5.f, 70.f, 70.f), Pose2f(defensiveStrikerPosition.angle(), defensiveStrikerPosition), 1);
          }
      }
  }

  state(turnToCenter)
  {
    transition
    { 
      if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle()) < 6_deg)
        goto stand;
    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(4.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle(), 0.f, 0.f), 1);
    }
  }

  state(stand)
  {
    transition
    {
      switch (theBehaviorStatus.role)
      {
        case Role::keeper:
          if(std::abs(keeperFinalPosition.x()) > offsetPosition &&
          std::abs(keeperFinalPosition.y()) > offsetPosition)
            goto moveKeeperToFinalStartPosition;
          break;
        case Role::rightDefender:
          if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
          {
            if(std::abs(offensiveRightDefenderPosition.x()) > offsetPosition &&
            std::abs(offensiveRightDefenderPosition.y()) > offsetPosition)
              goto moveDefenderToStartPosition;
          }
          else
          {
            if(std::abs(defensiveRightDefenderPosition.x()) > offsetPosition &&
            std::abs(defensiveRightDefenderPosition.y()) > offsetPosition)
              goto moveDefenderToStartPosition;
          }
          break;
        case Role::leftDefender:
          if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
          {
            if(std::abs(offensiveLeftDefenderPosition.x()) > offsetPosition &&
            std::abs(offensiveLeftDefenderPosition.y()) > offsetPosition)
              goto moveDefenderToStartPosition;
          }
          else
          {
            if(std::abs(defensiveLeftDefenderPosition.x()) > offsetPosition &&
            std::abs(defensiveLeftDefenderPosition.y()) > offsetPosition)
              goto moveDefenderToStartPosition;
          }
          break;
        case Role::supporter:
          if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
          {
            if((std::abs(offensiveSupporterPosition.x()) > offsetPosition) &&
            (std::abs(offensiveSupporterPosition.y()) > offsetPosition))
              goto moveSupporterToStartPosition;
          }
          else
          {
            if(std::abs(defensiveSupporterPosition.x()) > offsetPosition &&
            std::abs(defensiveSupporterPosition.y()) > offsetPosition)
              goto moveSupporterToStartPosition;
          }
          break;
        case Role::striker:
          if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
          {
            if(std::abs(offensiveStrikerPosition.x()) > offsetPosition &&
            std::abs(offensiveStrikerPosition.y()) > offsetPosition)
              goto moveStrikerToStartPosition;
          }
          else
          {
            if(std::abs(defensiveStrikerPosition.x()) > offsetPosition &&
            std::abs(defensiveStrikerPosition.y()) > offsetPosition)
              goto moveStrikerToStartPosition;
          }
          break;
      }
    }
    action
    {
        Stand();
    }
  }

}