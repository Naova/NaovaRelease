/**
 * Fait alignBehindBallSlow et KickFar. Prend un angle pour la direction du kick
 */
option(AlignForKick, (float) angle)
{
  initial_state(reposition)
  {
    transition
    {
      if(std::abs(theBallModel.estimate.position.angle()) <= 90_deg)
      {
        if(theRobotPose.rotation >= 0)
        {
          goto repositionLeft;
        }
        else
        {
          goto repositionRight;
        }
      }
      else
      {
        if(theRobotPose.rotation >= 0)
        {
          goto repositionRight;
        }
        else
        {
          goto repositionLeft;
        }
      }
    }
    action
    {
      TrackBallModel();
    }
  }

  state(repositionRight)
  {
    transition
    {
      if(std::abs(theBallModel.estimate.position.angle() - angle) < 50_deg)
      {
        goto alignBehindBall;
      }
    }
    action
    {
      TrackBallModel();
      WalkToTarget(Pose2f(30.f, 30.f, 25.f), Pose2f(angle, theBallModel.estimate.position.x() - 280.f, theBallModel.estimate.position.y() - 80.f));
    }
  }

  state(repositionLeft)
  {
    transition
    {
      if(std::abs(theBallModel.estimate.position.angle() - angle) < 50_deg)
      {
        goto alignBehindBall;
      }
    }
    action
    {
      TrackBallModel();
      WalkToTarget(Pose2f(30.f, 30.f, 25.f), Pose2f(angle, theBallModel.estimate.position.x() - 280.f, theBallModel.estimate.position.y() + 80.f));
    }
  }
  
  // On s'aligne pour faire le kick
  state(alignBehindBall)
  {
    transition
    {
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 40.f, 70.f)
          && theLibCodeRelease.between(theBallModel.estimate.position.x(), 180.f, 220.f)
          && std::abs(angle) < 3_deg)
      {
        goto finished;
      }
    }
    action
    {
      TrackBallModel();
      WalkToTarget(Pose2f(20.f, 20.f, 15.f), Pose2f(angle, theBallModel.estimate.position.x() - 200.f, theBallModel.estimate.position.y() - 60.f), 1);
    }
  }

  target_state(finished) {}
}