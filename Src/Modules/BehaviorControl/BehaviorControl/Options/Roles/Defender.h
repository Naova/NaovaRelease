//
// Created by vincent on 18-03-05.
//

option(Defender)
{
  common_transition
  {
    if(theLibCodeRelease.nbOfStriker == 0 && theLibCodeRelease.nbOfSupporter == 0)
    {
        theLibCodeRelease.setMissingTeammate(true);
      if(theLibCodeRelease.timeSinceMissingTeammate > theLibCodeRelease.waitToChangeRole + theRobotInfo.number*500)
      {
          theLibCodeRelease.setMissingTeammate(false);
            goto abortedState;
      }
    }
    else
    {
        theLibCodeRelease.setMissingTeammate(false);
    }

//    if(theLibCodeRelease.closerToTheBall)
//    {
//        if(state_time > theRobotInfo.number*500)
//        {
//            goto walkToBall;
//        }
//    }
//          else{
//        goto alignToBall;
//    }

  }

  initial_state(start)
    {
        transition
        {
            if (state_time > 1000)
                goto turnToBall;
        }
        action
        {
            LookForward();
            Stand();
        }
    }

  state(turnToBall)
  {
      transition
      {
          if(theTeamBallModel.isValid)
          {
              if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
              {
                  goto alignToBall;
              }
          }
      }
      action
      {
          LookForward();
          if(theTeamBallModel.isValid)
          {
              WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
          }
          else
          {
              //WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
          }
      }
  }

  aborted_state(abortedState){}

  state(alignToBall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto turnToBall;
            }

            if(theLibCodeRelease.closerToTheBall)
                goto alignToGoal;
            //si la balle zone danger
            //si la balle sort de la zone de position
        }
        action
        {
//            if (theLibCodeRelease.isLeftOrRightDefender()) {
                LookForward();

                WalkToTarget(Pose2f(90.f, 90.f, 90.f), Pose2f(theBallModel.estimate.position.angle(), theLibCodeRelease.desiredPos.x(), theLibCodeRelease.desiredPos.y())); // si def 1 ou 2, se pos par rapport a 1 ou 2
//            }else{

//            }
        }
    }
  state(alignToGoal)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
          {
              goto turnToBall;
          }
          if(std::abs(theLibCodeRelease.angleToOppGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
          {
              goto alignBehindBall;
          }
          if(theBallModel.estimate.position.norm() > 600.f)
          {
              goto alignToBall;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
      }
  }

  state(walkToBall)
  {
      transition
      {
          if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
          {
              goto turnToBall;
          }
          if(theBallModel.estimate.position.norm() < 500.f)
          {
              goto alignToGoal;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), theRobotPose.inversePose * theTeamBallModel.position));
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
//          if(std::abs(theBallModel.estimate.position.y())<100.f
//             && std::abs(theBallModel.estimate.position.x())< 350.f
//             && std::abs(theLibCodeRelease.angleToDesiredKick) < 20_deg)
//          {
//              goto alignBehindBallSlow;
//          }
          else if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
                  && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
                  && std::abs(theLibCodeRelease.angleToOppGoal) < 2_deg)
          {
              goto kick;
          }
          else if(theBallModel.estimate.position.norm() > 600.f)
          {
              goto alignToBall;
          }
      }
      action
      {
          LookForward();
          WalkToTarget(Pose2f(40.f, 40.f, 40.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
      }
  }
  state(kick)
  {
      transition
      {
          if(state_time > 3000 || (state_time > 10 && action_done))
          {
              goto start;
          }
          if(theBallModel.estimate.position.norm() > 600.f)
          {
              goto alignToBall;
          }
          if(action_done){
              goto alignToBall;
          }
      }
      action
      {
          LookForward();
          theMotionRequest.motion = MotionRequest::kick;
          theMotionRequest.kickRequest.kickMotionType = KickRequest::kickForward;
          theMotionRequest.kickRequest.mirror = true;
//          InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToDesiredKick, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
      }
  }


}
