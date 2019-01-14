/** behavior for the ready state */

option(ReadyState)
{
    initial_state(ready)
    {
        transition{
            goto startRoleAttribution;
        }

    }

  state(startRoleAttribution)
  {
    transition
    {
        if(action_done)
            goto turnToIdealPosition;
    }
    action
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
        LookForward();
        Stand();

    }
  }

  state(moveKeeperToStartPosition)
  {
      transition
      {
          if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline + theLibCodeRelease.safeDistance,0)).x()) < 5 &&
              std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline + theLibCodeRelease.safeDistance,0)).y()) < 5)
              goto turnToCenter;
      }
      action
      {
        theBehaviorStatus.role = Role::keeper;
        WalkToTarget(Pose2f(100.f, 100.f, 100.f),Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle(),
                                                        (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline + theLibCodeRelease.safeDistance,0)).x(),
                                                        (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline + theLibCodeRelease.safeDistance,0)).y()));
      }
  }

  state(moveDefenderToStartPosition)
  {
      transition
      {
          if(theRobotInfo.number == 3){
              if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoal / 2.f)).x()) < 5 &&
                 std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoal / 2.f)).y()) < 5)
                  goto turnToCenter;
          }
          else
          {
              if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoal / 2.f)).x()) < 5 &&
                 std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoal / 2.f)).y()) < 5)
                  goto turnToCenter;
          }
      }
      action
      {
        theBehaviorStatus.role = Role::defender;
        if(theRobotInfo.number == 3)
            WalkToTarget(Pose2f(100.f, 100.f, 100.f),Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance, theFieldDimensions.yPosLeftGoal / 2.f)).angle(),
                                                          (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoal / 2.f)).x(),
                                                          (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosLeftGoal / 2.f)).y()));
        else
            WalkToTarget(Pose2f(100.f, 100.f, 100.f),Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance, theFieldDimensions.yPosRightGoal / 2.f)).angle(),
                                                          (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoal / 2.f)).x(),
                                                          (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,theFieldDimensions.yPosRightGoal / 2.f)).y()));
      }
  }

    state(moveStrikerToStartPosition)
    {
        transition
        {
            if(theGameInfo.kickingTeam == Global::getSettings().teamNumber){
                if(std::abs((theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 200,0)).x()) < 10 &&
                   std::abs((theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 200,0)).y()) < 10)
                    goto turnToCenter;
            }
            else
            {
                if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance * 6, 0)).x()) < 10 &&
                   std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance * 6, 0)).y()) < 10)
                    goto turnToCenter;
            }


        }
        action
        {
            theBehaviorStatus.role = Role::striker;
            if(theGameInfo.kickingTeam == Global::getSettings().teamNumber)
                WalkToTarget(Pose2f(100.f, 100.f, 100.f),Pose2f((theRobotPose.inversePose * Vector2f( -theLibCodeRelease.footLength - 200, 0.f)).angle(),
                                                                (theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 200, 0)).x(),
                                                                (theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 200, 0)).y()));
            else
                WalkToTarget(Pose2f(100.f, 100.f, 100.f),Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance * 6, 0)).angle(),
                                                                (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance * 6, 0)).x(),
                                                                (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance * 6, 0)).y()));
        }
    }

  state(moveSupporterToStartPosition)
  {
      transition
      {
          if((std::abs((theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 500, theFieldDimensions.yPosRightGoal)).x()) < 5) &&
              (std::abs((theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 500, theFieldDimensions.yPosRightGoal)).y()) < 5))
          {
              goto turnToCenter;
          }
          else if ((std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance  * 10, theFieldDimensions.yPosRightGoal)).x()) < 5) &&
              (std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance  * 10, theFieldDimensions.yPosRightGoal)).y()) < 5))
          {goto turnToCenter;}
      }
      action
      {
        theBehaviorStatus.role = Role::supporter;

        if(theGameInfo.kickingTeam == Global::getSettings().teamNumber) {
            WalkToTarget(Pose2f(100.f, 100.f, 100.f),
                         Pose2f((theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 500, theFieldDimensions.yPosRightGoal)).angle(),
                                (theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 500, theFieldDimensions.yPosRightGoal)).x(),
                                (theRobotPose.inversePose * Vector2f(-theLibCodeRelease.footLength - 500, theFieldDimensions.yPosRightGoal)).y()));
        }
        else
        {
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * Vector2f(
                    theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance  * 10,
                    theFieldDimensions.yPosRightGoal)).angle(),
                                                             (theRobotPose.inversePose * Vector2f(
                                                                     theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance  * 10,
                                                                     theFieldDimensions.yPosRightGoal)).x(),
                                                             (theRobotPose.inversePose * Vector2f(
                                                                     theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance  * 10,
                                                                     theFieldDimensions.yPosRightGoal)).y()));
        }
      }
  }

  state(stand)
  {
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
          if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle()) < 6_deg)
              goto stand;
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(4.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle(), 0.f, 0.f));
      }
  }

  state(turnToIdealPosition)
  {
      transition
      {
          if(theRobotInfo.number == 1) {
              if (std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle()) <
                  5_deg) {
                  goto moveKeeperToStartPosition;
              }
          }
          if(theRobotInfo.number == 2) {
              if (theGameInfo.kickingTeam == Global::getSettings().teamNumber
                  && std::abs((theRobotPose.inversePose *
                            Vector2f(-theFieldDimensions.centerCircleRadius - theLibCodeRelease.footLength,
                                     0.f)).angle()) < 10_deg) {
                  goto moveStrikerToStartPosition;
              } else if (theGameInfo.kickingTeam != Global::getSettings().teamNumber &&
                         std::abs((theRobotPose.inversePose *
                                   Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,
                                            theFieldDimensions.yPosLeftPenaltyArea)).angle()) < 10_deg) {
                  goto moveStrikerToStartPosition;
              }
          }
          if(theRobotInfo.number == 3) {
              if (std::abs((theRobotPose.inversePose *
                            Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,
                                     theFieldDimensions.yPosLeftGoal / 2.f)).angle()) < 10_deg)
                  goto moveDefenderToStartPosition;
          }if(theRobotInfo.number == 4)
          {       if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance, theFieldDimensions.yPosRightGoal / 2.f)).angle()) < 10_deg) {
                  goto moveDefenderToStartPosition;
              }
          }
          if(theRobotInfo.number == 5)
          {        if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightGoal)).angle()) < 10_deg) {
                  goto moveSupporterToStartPosition;
              }
          }
      }
      action
      {
          LookForward();
          if(theRobotInfo.number == 1)
              WalkToTarget(Pose2f(5.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle(), 0.f, 0.f));
          if(theRobotInfo.number == 2) {
              if (theGameInfo.kickingTeam == Global::getSettings().teamNumber)
                  WalkToTarget(Pose2f(5.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(
                          -theFieldDimensions.centerCircleRadius - theLibCodeRelease.footLength, 0.f)).angle(), 0.f,
                                                             0.f));
              else
                  WalkToTarget(Pose2f(5.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(
                          theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance,
                          theFieldDimensions.yPosLeftPenaltyArea)).angle(), 0.f, 0.f));
          }
          if(theRobotInfo.number == 3)
              WalkToTarget(Pose2f(5.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance, theFieldDimensions.yPosLeftGoal / 2.f)).angle(), 0.f, 0.f));
          if(theRobotInfo.number == 4)
              WalkToTarget(Pose2f(5.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyArea + theLibCodeRelease.safeDistance, theFieldDimensions.yPosRightGoal / 2.f)).angle(), 0.f, 0.f));
          if(theRobotInfo.number == 5)
              WalkToTarget(Pose2f(5.f, 5.f, 5.f), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosRightGoal)).angle(), 0.f, 0.f));
      }
  }
}
