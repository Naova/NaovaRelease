// We May need 2 state machine for looking at the ball and looking at a point because
// The way we calculate the target may vary from a point and a ball

option(TrackBallModel, (float)(pi) speed, ((HeadMotionRequest) CameraControlMode)(autoCamera) camera, (bool)(false) stopAndGoMode)
{
  // Starts tracking the Point
  initial_state(startTracking)
  {
    transition
    {
      // If the intent is to track the ball, we start looking for it 
      if(!theLibCodeRelease.ballModelIsValid)
        goto startSearchingBall;
    }
    action
    {
      theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
      theHeadMotionRequest.cameraControlMode = camera;
      theHeadMotionRequest.speed = speed;

      Pose2f pose = theBallModel.estimate.position;
      theHeadMotionRequest.target = Vector3f(pose.translation.x(), pose.translation.y(), 0.f);
      theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
    }
  }

  /** This state waits until the SearchBall control finds the ball */
  state(startSearchingBall)
  {
    transition
    {
      if(theTeamBallModel.isValid)
        goto startTracking;
    }
    action
    {
    }
  }
}
