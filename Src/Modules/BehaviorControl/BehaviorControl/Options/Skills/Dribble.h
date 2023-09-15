/**
 * Sets all members of the MotionRequest representation for executing an absoluteSpeedMode-WalkRequest
 * (i.e. Walk at a \c speed)
 * @param speed Walking speeds, in mm/s and radian/s.
 *              e.g.  Pose2f(0.f, 50.f, 0.f) lets move the robot forward at 50 mm/s
 *                    Pose2f(0.f, 50.f, 50.f) lets move the robot diagonal
 *                    Pose2f(0.2f, 100.f, 0.f) lets move the robot in a circle
 */
option(Dribble)//, (const Pose2f&) target)
{
  /** Set the motion request. */
  initial_state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::walk)
        goto requestIsExecuted;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
      theMotionRequest.walkRequest.speed = Pose2f(theDribblingQLearning.Vr, theDribblingQLearning.Vx, theDribblingQLearning.Vy);
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    }
  }

  /** The motion process has started executing the request. */
  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::walk)
        goto setRequest;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
      theMotionRequest.walkRequest.speed = Pose2f(theDribblingQLearning.Vr, theDribblingQLearning.Vx, theDribblingQLearning.Vy);
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    }
  }
}
