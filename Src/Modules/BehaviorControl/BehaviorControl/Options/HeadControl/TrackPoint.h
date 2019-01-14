// make the head of the Nao track a point with its head

option(TrackPoint, (Vector2f) point, (float)(pi) speed, ((HeadMotionRequest) CameraControlMode)(autoCamera) camera, (bool)(false) stopAndGoMode)
{
  // Starts tracking the Point
  initial_state(startTracking)
  {
    transition
    {

    }
    action
    {
      theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
      theHeadMotionRequest.cameraControlMode = camera;
      theHeadMotionRequest.speed = speed;

      Pose2f pose = theRobotPose.inversePose * theTeamBallModel.position;
      theHeadMotionRequest.target = Vector3f(pose.translation.x(), pose.translation.y(), 0.f);
      //theHeadMotionRequest.target = Vector3f(point.translation.x() + 500.f, point.translation.y(), 0.f);
      theHeadMotionRequest.stopAndGoMode = stopAndGoMode;
    }
  }
}
