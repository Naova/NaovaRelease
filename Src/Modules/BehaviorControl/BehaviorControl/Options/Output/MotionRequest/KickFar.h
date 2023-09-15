/**
 * Fait alignBehindBallSlow et KickFar. Prend un angle pour la direction du kick
 */
option(KickFar, (float) angle)
{
  // On s'aligne pour faire le kick
  initial_state(alignBehindBall)
  {
    transition
    {
      if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 38.f, 62.f)
          && theLibCodeRelease.between(theBallModel.estimate.position.x(), 152.f, 177.f)
          && std::abs(angle) < 3_deg)
      {
        goto kickFar;
      }
    }
    action
    {
      TrackBallModel();
      WalkToTarget(Pose2f(0.5f, 0.5f, 0.5f), Pose2f(angle, theBallModel.estimate.position.x() - 165.f, theBallModel.estimate.position.y() - 50.f));
    }
  }

  // On fait le kick
  state(kickFar)
  {
    transition
    {
      if(state_time > 3000 || (state_time > 10 && action_done))
      {
        goto finished;
      }
    }
    action
    {
      LookForward();
      theMotionRequest.motion = MotionRequest::kick;
      theMotionRequest.kickRequest.kickMotionType = KickRequest::kickForward;
      theMotionRequest.kickRequest.mirror = true;
    }
  }

  // Le kick est fini
  target_state(finished) {}

}
