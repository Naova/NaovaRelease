/** Sets all members of the MotionRequest representation for executing a targetMode-WalkRequest
 *  (i.e. Walk to a \c target at a \c speed)
 *  @param speed Walking speeds, as a ratio of the maximum speeds [0..1].
 *  @param target Walking target, in mm and radians, relative to the robot.
 */
option(WalkToTarget, (const Pose2f&) speed, (const Pose2f&) target, (bool)(0) realign)
{
  initial_state(realignToTarget)
  {
    transition
    {
      if(std::abs(target.rotation) < 2_deg) {
        if (realign && target.translation.norm() > 1200.f)
          goto walkFar;
        else
          goto requestIsExecuted;
      }
      else if (!realign)
      {
        goto requestIsExecuted;
      }
    }
    action
    { 
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = Pose2f(target.rotation, 0.f, 0.f);
      theMotionRequest.walkRequest.speed = speed;
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();      
    }
  }

  state(walkFar)
  {
    transition
    {
      if(target.translation.norm() < 1000)
        goto requestIsExecuted;
      if(realign && std::abs(target.rotation) > 40_deg && target.translation.norm() > 600.f)
        goto realignToTarget;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::relativeSpeedMode;
      theMotionRequest.walkRequest.speed = Pose2f(0.f, (float)speed.translation.x() / 100.f, 0.f);
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    }
  }

  state(setRequest)
  {
    transition
    {
      if(theMotionInfo.motion == MotionRequest::walk)
        goto requestIsExecuted;
      if(realign && std::abs(target.rotation) > 60_deg && target.translation.norm() > 600.f)
        goto realignToTarget;
      if (realign && target.translation.norm() > 1200.f)
        goto walkFar;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = target;
      theMotionRequest.walkRequest.speed = speed;
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();

      
    }
  }

  target_state(requestIsExecuted)
  {
    transition
    {
      if(theMotionInfo.motion != MotionRequest::walk)
        goto setRequest;
      if(realign && std::abs(target.rotation) > 60_deg && target.translation.norm() > 600.f)
        goto realignToTarget;
      if (realign && target.translation.norm() > 1200.f)
        goto walkFar;
    }
    action
    {
      theMotionRequest.motion = MotionRequest::walk;
      theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
      theMotionRequest.walkRequest.target = target;
      theMotionRequest.walkRequest.speed = speed;
      theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();

      
    }
  }
}