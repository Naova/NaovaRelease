/** A test striker option without common decision */
option(Striker)
{
    common_transition
    {
        if(!theLibCodeRelease.closerToTheBall)
        {
          if(state_time > theRobotInfo.number*500)
          {
            goto abortedState;
          }
        }
    }

    initial_state(start)
    {
      transition
      {
        if(state_time > 1000)
          goto turnToBall;
      }
      action
      {
          TrackBall();
        Stand();
      }
    }

    aborted_state(abortedState){}

    state(turnToBall)
    {
      transition
      {
        if(theTeamBallModel.isValid)
        {
            if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
            {
                goto walkToBall;
            }
        }
          if(state_time > 5000)
          {
              goto walkToLastBall;
          }
      }
      action
      {
          TrackBall();
          if(theTeamBallModel.isValid)
          {
              WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
          }
          else
          {
              WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
          }
      }
    }

    state(walkToLastBall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen < theBehaviorParameters.ballNotSeenTimeOut)
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
            TrackBall();
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), theRobotPose.inversePose * theTeamBallModel.position));
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
        TrackBall();
        WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), theRobotPose.inversePose * theTeamBallModel.position));
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
          goto walkToBall;
        }
      }
      action
      {
        TrackBall();
        WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
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
      if(std::abs(theBallModel.estimate.position.y())<100.f
         && std::abs(theBallModel.estimate.position.x())< 350.f
         && std::abs(theLibCodeRelease.angleToDesiredKick) < 20_deg &&
              theRobotPose.translation.x() > theFieldDimensions.xPosOpponentGroundline*0.75 )
      {
          goto alignBehindBallSlow;
      }
        else if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
           && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
           && std::abs(theLibCodeRelease.angleToDesiredKick) < 2_deg &&
              theRobotPose.translation.x() <= theFieldDimensions.xPosOpponentGroundline*0.75)
        {
          goto kick;
        }
        if(theBallModel.estimate.position.norm() > 600.f)
        {
          goto walkToBall;
        }
      }
      action
      {
        TrackBall();
        WalkToTarget(Pose2f(40.f, 40.f, 40.f), Pose2f(theLibCodeRelease.angleToDesiredKick, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
      }
    }

    state(alignBehindBallSlow)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto turnToBall;
            }
            if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 35.f, 50.f)
               && theLibCodeRelease.between(theBallModel.estimate.position.x(), 165.f, 180.f)
               && std::abs(theLibCodeRelease.angleToDesiredKick) < 5_deg &&
               theRobotPose.translation.x() > theFieldDimensions.xPosOpponentGroundline*0.75 )
            {
                goto kickFar;
            }
            if(theBallModel.estimate.position.norm() > 600.f)
            {
                goto walkToBall;
            }
        }
        action
        {
            TrackBall();
            WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(theLibCodeRelease.angleToDesiredKick, theBallModel.estimate.position.x() - 170.f, theBallModel.estimate.position.y() - 45.f));
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
          goto walkToBall;
        }
      }
      action
      {
          TrackBall();
        InWalkKick(WalkKickVariant(WalkKicks::forwardShoot, Legs::left), Pose2f(theLibCodeRelease.angleToDesiredKick, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
      }
    }

    state(kickFar)
    {
        transition
        {
            if(state_time > 3000 || (state_time > 10 && action_done))
            {
                goto start;
            }
            if(theBallModel.estimate.position.norm() > 600.f)
            {
                goto walkToBall;
            }
        }
        action
        {
            LookForward();
//            InWalkKick(WalkKickVariant(WalkKicks::forwardShoot, Legs::left), Pose2f(theLibCodeRelease.angleToDesiredKick, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
            theMotionRequest.motion = MotionRequest::kick;
            theMotionRequest.kickRequest.kickMotionType = KickRequest::kickForward;
            theMotionRequest.kickRequest.mirror = true;
        }
    }
}
