/** A test striker option without common decision */
option(StrikerPenaltyShoot)
{
    initial_state(start)
    {
      transition
      {
        if(state_time > 1000)
          goto searchForBall;
      }
      action
      {
        LookForward();
        Stand();
      }
    }

    state(alignBehindBall)
    {
      transition
      {
        if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut)
        {
          goto searchForBall;
        }
        if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
           && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
           && std::abs(theLibCodeRelease.angleToDesiredKick) < 2_deg)
        {
          goto kick;
        }
      }
      action
      {
        LookForward();
        WalkToTarget(Pose2f(0.35f, 0.35f, 0.35f), Pose2f(theLibCodeRelease.angleToDesiredKick, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
      }
    }

    state(kick)
    {
      transition
      {
        if(state_time > 3000 || (state_time > 10 && action_done))
        {
          goto stand;
        }
      }
      action
      {
        LookForward();
        InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(-theLibCodeRelease.angleToDesiredKick, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
      }
    }

    state(searchForBall)
    {
      transition
      {
        if(theLibCodeRelease.timeSinceBallWasSeen < 300)
        {
          goto alignBehindBall;
        }
      }
      action
      {
        LookForward();
        WalkAtRelativeSpeed(Pose2f(0.f, 0.1f, 0.f));
      }
    }

    state(stand)
    {
        action
        {
            Stand();
        }
    }
}
