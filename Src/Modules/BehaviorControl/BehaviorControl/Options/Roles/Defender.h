//
// Created by vincent on 18-03-05.
//

option(Defender)
{
    initial_state(start)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;

            if (state_time > 1000)
                goto turnToBall;
        }
        action
        {
            TrackBall();
            Stand();
        }
    }

    aborted_state(abortedState)
    {
        transition
        {
            if (theBehaviorStatus.role == Role::defender)
                goto start;
        }
    }

    state(turnToBall)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;

            if(theTeamBallModel.isValid)
            {
                if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
                {
                    goto alignToBall;
                }
            }
            else
            {
                goto SearchBall;
            }
        }
        action
        {
            TrackBall();
            if(theTeamBallModel.isValid)
            {
                WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
            }
        }
    }

    state(SearchBall)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;

            if (theTeamBallModel.isValid)
            {
                goto turnToBall;
            }
        }
        action
        {
            SearchBall();
        }
    }

    state(alignToBall)
    {
        transition
        {
            if (theTeamData.canSendMessages && theRoleChanges.nextRole != theBehaviorStatus.role)
                goto abortedState;

            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
            {
                goto turnToBall;
            }

            if(theLibCodeRelease.closerToTheBall || (theRobotPose.inversePose*theTeamBallModel.position).norm() <= theLibCodeRelease.defenderDistanceForKick)
                goto alignToGoal;
            //si la balle zone danger
            //si la balle sort de la zone de position
        }
        action
        {
            TrackBall();

            WalkToTarget(Pose2f(90.f, 90.f, 90.f), Pose2f(theBallModel.estimate.position.angle(), theLibCodeRelease.desiredPos.x(), theLibCodeRelease.desiredPos.y())); // si def 1 ou 2, se pos par rapport a 1 ou 2
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
            if(!theLibCodeRelease.closerToTheBall && (theRobotPose.inversePose*theTeamBallModel.position).norm() > theLibCodeRelease.defenderDistanceForKick)
            {
                goto alignToBall;
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
            // if(std::abs(theBallModel.estimate.position.y())<100.f
            // && std::abs(theBallModel.estimate.position.x())< 350.f
            // && std::abs(theLibCodeRelease.angleToDesiredKick) < 20_deg)
            // {
            //     goto alignBehindBallSlow;
            // }
            else if(std::abs(theLibCodeRelease.angleToOppGoal) > 10_deg && std::abs(theBallModel.estimate.position.y()) > 100.f)
            {
                goto alignToGoal;
            }
            else if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
                    && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
                    && std::abs(theLibCodeRelease.angleToOppGoal) < 5_deg)
            {
                goto kick;
            }
            else if(!theLibCodeRelease.closerToTheBall && (theRobotPose.inversePose*theTeamBallModel.position).norm() > theLibCodeRelease.defenderDistanceForKick)
            {
                goto alignToBall;
            }
        }
        action
        {
            TrackBall();
            WalkToTarget(Pose2f(40.f, 40.f, 40.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
        }
    }

  state(kick)
  {
      transition
      {
        if(state_time > 10 && action_done)
        {
                goto start;
        }
        
        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        {
            goto turnToBall;
        }

        if(!theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
                && !theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
                && std::abs(theLibCodeRelease.angleToOppGoal) > 5_deg)
        {
            goto alignBehindBall;
        }

        if(!theLibCodeRelease.closerToTheBall && (theRobotPose.inversePose*theTeamBallModel.position).norm() > theLibCodeRelease.defenderDistanceForKick)
        {
            goto alignToBall;
        }

        if(action_done)
        {
            goto alignToBall;
        }
      }
      action
      {
        Vector2f position = Vector2f(0,0);
        
        if(theTeamBallModel.position.y() >= 0)
        {
            position = Vector2f(theFieldDimensions.xPosHalfWayLine,theFieldDimensions.centerCircleRadius);
        }
        else
        {
            position = Vector2f(theFieldDimensions.xPosHalfWayLine,-theFieldDimensions.centerCircleRadius);
        }

        KickWithPosition(position);
      }
  }
}
