option(TrackBallSweep, (float)(pi) speed, ((HeadMotionRequest) CameraControlMode)(autoCamera) camera, (bool)(false) stopAndGoMode)
{
  // Starts tracking the Point
  initial_state(startTracking)
  {
    transition
    {
      // If the intent is to track the ball, we start looking for it 
      if(!theTeamBallModel.isValid)
        goto moveSwitchRight;
    }
    action
    {
      theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
      theHeadMotionRequest.cameraControlMode = camera;
      theHeadMotionRequest.speed = speed;

      Pose2f pose = theRobotPose.inversePose * theTeamBallModel.position;
      theHeadMotionRequest.target = Vector3f(pose.translation.x(), pose.translation.y(), 0.f);
      theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
    }
  }

    /** Moves the head to the far right, then change the state to moveSwitchLeft */
  state(moveSwitchRight)
  {
     transition
    {
		  if(theTeamBallModel.isValid)
        goto startTracking;
      else if(!theHeadMotionEngineOutput.moving && theHeadMotionEngineOutput.pan <= (-pi/4))
        goto moveSwitchLeft;
    }
    action
    {
		  SetHeadPanTilt((-pi/4)*1.2, 1.5,pi*0.4);
    }
  }

  /** Moves the head to the far left, if the ball has still not been found, we enter aborted_state */
  state(moveSwitchLeft)
  {
     transition
    {
      if(theTeamBallModel.isValid)
        goto startTracking;
      else if(!theHeadMotionEngineOutput.moving && theHeadMotionEngineOutput.pan >= (pi/4))
        goto moveSwitchRight;
    }
    action
    {
      SetHeadPanTilt((pi/4)*1.2, 1.5,pi*0.4);
    }
  }
}
