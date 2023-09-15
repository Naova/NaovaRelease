option(DefensivePenaltyShoot)
{
  common_transition
  {
      /*if(theLibCodeRelease.between(theTeamBallModel.position.x(),200.f,600.f)
         && theLibCodeRelease.between(theTeamBallModel.position.y(),0.f,1000.f)
         && theLibCodeRelease.between(theTeamBallModel.velocity.x(),5.f,1000.f))
         goto diveLeft;

      if(theLibCodeRelease.between(theTeamBallModel.position.x(),200.f,600.f)
         && theLibCodeRelease.between(theTeamBallModel.position.y(),0.f,-1000.f)
         && theLibCodeRelease.between(theTeamBallModel.velocity.x(),5.f,1000.f))
         goto diveRight;
      */

	if (theGroundContactState.contact  
    && !theLibCodeRelease.ballIsInGoalArea
    && theLibCodeRelease.between(theBallModel.estimate.position.y(), -200.f, 200.f)
    && theLibCodeRelease.between(theBallModel.estimate.velocity.x(), -1000.f, -50.f)
    && theLibCodeRelease.between(theBallModel.estimate.position.x(), 0.f, 1000.f))
       goto sumo;

	if(theGroundContactState.contact
    && !theLibCodeRelease.ballIsInGoalArea
    && theLibCodeRelease.between(theBallModel.estimate.position.x(), 0, 1000.f)
    && theLibCodeRelease.between(theBallModel.estimate.position.y(), 200.f, 600.f)
    && theLibCodeRelease.between(theBallModel.estimate.velocity.x(), -1000.f, -50.f))
        goto groundPunchLeft;


	if(theGroundContactState.contact
    && !theLibCodeRelease.ballIsInGoalArea
    && theLibCodeRelease.between(theBallModel.estimate.position.x(), 0, 1000.f)
    && theLibCodeRelease.between(theBallModel.estimate.position.y(), -600.f, -200.f)
    && theLibCodeRelease.between(theBallModel.estimate.velocity.x(), -1000.f, -50.f))
        goto groundPunchRight;

	if(theLibCodeRelease.nbOfKeeper >= 2)
       if(state_time > theRobotInfo.number*100)
         goto abortedState;
 }

  initial_state(start)
  {
    transition
    {
      if(state_time > 1000)
      {
        goto alignToBall;
      }
    }
    action
    {
      LookForward();
      Stand();
    }
  }

  aborted_state(abortedState){}

//ba
//  state(diveLeft)
//  {
//    transition
//    {
//      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
//      {
//        goto start;
//      }
//    }
//    action
//    {
//      // ** Desactivated until the special action exist **
//      //SpecialAction(SpecialActionRequest::diveLeft);
//    }
//  }
//
//  state(diveRight)
//  {
//    transition
//    {
//      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
//      {
//        goto start;
//      }
//    }
//    action
//    {
//      // ** Desactivated until the special action exist **
//      //SpecialAction(SpecialActionRequest::diveRight);
//    }
//  }
//
  state(sumo)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
      {
        goto start;
      }
    }
    action
    {
        SpecialAction(SpecialActionRequest::sumo);
        
    }
  }

state(groundPunchLeft)
 {
   transition
   {
     if(state_time > 10 && action_done)
     {
       goto start;
     }
   }
   action
   {
    SpecialAction(SpecialActionRequest::groundPunchLeft);
   }
 }

state(groundPunchRight)
 {
   transition
   {
     if(state_time > 10 && action_done)
     {
       goto start;
     }
   }
   action
   {
    SpecialAction(SpecialActionRequest::groundPunchRight);
   }
 }

    state(walkToBall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                if((theRobotPose.inversePose * theTeamBallModel.velocity).y() > 0)
                {
                    goto searchLeftForBall;
                }
                else if((theRobotPose.inversePose * theTeamBallModel.velocity).y() < 0)
                {
                    goto searchRightForBall;
                }
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

        state(alignToGoal)
        {
            transition
            {
                if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
                {
                    if((theRobotPose.inversePose * theTeamBallModel.velocity).y() > 0)
                    {
                        goto searchLeftForBall;
                    }
                    else if((theRobotPose.inversePose * theTeamBallModel.velocity).y() < 0)
                    {
                        goto searchRightForBall;
                    }
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
                WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 200.f, theBallModel.estimate.position.y()));
            }
        }

  state(alignToBall)
  {
    transition
    {
        if(theLibCodeRelease.ballIsInGoalArea)
            goto walkToBall;

        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        {
          if((theRobotPose.inversePose * theTeamBallModel.velocity).y() > 0)
          {
            goto searchLeftForBall;
          }
          else if((theRobotPose.inversePose * theTeamBallModel.velocity).y() < 0)
          {
            goto searchRightForBall;
          }
        }
    }
    action
    {
      WalkToTarget(Pose2f(90.f, 90.f, 90.f), Pose2f(theBallModel.estimate.position.angle(), theLibCodeRelease.desiredPos));
    }
  }

  state(turnToBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
      {
          if((theRobotPose.inversePose * theTeamBallModel.velocity).y() > 0)
          {
            goto searchLeftForBall;
          }
          else if((theRobotPose.inversePose * theTeamBallModel.velocity).y() < 0)
          {
            goto searchRightForBall;
          }
      }
      if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
      {
       goto alignToBall;
      }

    }
    action
    {
      LookForward();
      WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
    }
  }

        state(alignBehindBall)
        {
            transition
            {
                if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
                {
                    if((theRobotPose.inversePose * theTeamBallModel.velocity).y() > 0)
                    {
                        goto searchLeftForBall;
                    }
                    else if((theRobotPose.inversePose * theTeamBallModel.velocity).y() < 0)
                    {
                        goto searchRightForBall;
                    }
                }
                if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
                   && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
                   && std::abs(theLibCodeRelease.angleToOppGoal) < 2_deg)
                {
                    goto kick;
                }
                if(theBallModel.estimate.position.norm() > 600.f)
                {
                    goto alignToBall;
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
            if(state_time > 3000 || (state_time > 10 && action_done))
            {
                goto start;
            }
            if(theBallModel.estimate.position.norm() > 600.f)
            {
                goto alignToBall;
            }
        }
        action
        {
            LookForward();
            InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
        }
    }

  state(searchLeftForBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
      {
        goto turnToBall;
      }
      if(state_time > 3000)
	    goto start;
    }
    action
    {
      LookForward();
      WalkAtRelativeSpeed(Pose2f(1.f, 0.f, 0.f));
    }
  }

  state(searchRightForBall)
  {
    transition
    {
      if(theLibCodeRelease.timeSinceBallWasSeen < 300)
      {
        goto turnToBall;
      }
      if(state_time > 3000)
	    goto start;
    }
    action
    {
      LookForward();
      //WalkAtRelativeSpeed(Pose2f(-1.f, 0.f, 0.f));
    }
  }
}
