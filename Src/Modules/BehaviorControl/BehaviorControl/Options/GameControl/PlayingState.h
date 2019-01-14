option(PlayingState)
{
  initial_state(playing)
  {
      transition
      {
          if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT) {
              goto penaltyShoot;
          }

          if (theGameInfo.setPlay == SET_PLAY_NONE)
          {
              goto teamTactic;
          }

          else if (theGameInfo.setPlay == SET_PLAY_GOAL_FREE_KICK)
          {
              if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
              {
                  if(theRobotInfo.number == 1)
                  {
                      goto turnAndKick;
                  }
                  else
                  {
                      goto standUp;
                  }
              }
              else
              {
                  goto turnAndBack;
              }
          }
          else if (theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK)
          {
              if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber)
              {
                  if(theLibCodeRelease.closerToTheBall)
                  {
                      goto turnAndKick;
                  }
                  else
                  {
                      goto turnAndBack;
                  }
              }
              else
              {
                  goto turnAndBack;
              }
          }
      }
  }
  state(penaltyShoot)
  {
      transition
      {
      }
      action
      {
          if(theGameInfo.kickingTeam != Global::getSettings().teamNumber){
              KeeperPenaltyShoot();
          }else {
              StrikerPenaltyShoot();
          }

      }
  }

  state(teamTactic)
  {
      transition
      {
          if(theGameInfo.setPlay != SET_PLAY_NONE)
          {
              goto playing;
          }
      }
      action
      {
          HandleTeamTactic();
      }
  }

  state(turnAndBack)
  {
      transition
      {
          if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
          {
              goto backOff;
          }
          if(theGameInfo.setPlay == SET_PLAY_NONE)
          {
              goto playing;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
      }
  }

  state(backOff)
  {
      transition
      {
          if((theRobotPose.inversePose * theTeamBallModel.position).norm() > 760.f)
          {
              goto standUp;
          }
          if(theGameInfo.setPlay == SET_PLAY_NONE)
          {
              goto playing;
          }
      }
      action
      {
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), -200.f, 0.f));
      }
  }

  state(turnAndKick)
  {
      transition
      {
          if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
          {
              goto walkToBall;
          }
          if(theGameInfo.setPlay == SET_PLAY_NONE)
          {
              goto playing;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
      }
  }

  state(walkToBall)
  {
      transition
      {
          if(theBallModel.estimate.position.norm() < 500.f)
          {
              goto alignToGoal;
          }
          if(theGameInfo.setPlay == SET_PLAY_NONE)
          {
              goto playing;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(100.f, 100.f, 100.f), theRobotPose.inversePose * theTeamBallModel.position);
      }
  }

  state(alignToGoal)
  {
      transition
      {
          if(std::abs(theLibCodeRelease.angleToOppGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
          {
              goto alignBehindBall;
          }
          if(theBallModel.estimate.position.norm() > 600.f)
          {
              goto walkToBall;
          }
          if(theGameInfo.setPlay == SET_PLAY_NONE)
          {
              goto playing;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
      }
  }

  state(alignBehindBall)
  {
      transition
      {
          if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
             && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
             && std::abs(theLibCodeRelease.angleToOppGoal) < 2_deg)
          {
              goto kick;
          }
          if(theBallModel.estimate.position.norm() > 600.f)
          {
              goto walkToBall;
          }
          if(theGameInfo.setPlay == SET_PLAY_NONE)
          {
              goto playing;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
      }
  }

  state(kick)
  {
      transition
      {
          if(theBallModel.estimate.position.norm() > 600.f || state_time > 3000 || (state_time > 10 && action_done))
          {
              goto walkToBall;
          }
          if(theGameInfo.setPlay == SET_PLAY_NONE)
          {
              goto playing;
          }
      }
      action
      {
          LookForward();
          InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
      }
  }

  state(standUp)
  {
      transition
      {
        if(theGameInfo.setPlay == SET_PLAY_NONE)
        {
            goto playing;
        }
      }
      action
      {
          LookForward();
          Stand();
      }
  }
}