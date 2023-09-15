/* Search for the ball if the player is a striker and if the teamball is not valid
*  We start by moving in direction of the opposit side line, then we move in direction of the goal and then we come back at the first point
*  If the ball doesn't get found, we enter aborted_state and we start again the triangle
*/
option(SearchBallTriangle)
{
  //We define the distance of walk as one quarter of the width of the field
  float walkDistance = theFieldDimensions.yPosLeftSideline / 2;

  /** Starts moving to the right */
  initial_state(StartSearchTriangle)
  {
    transition
    {
      goto triangleSearch;
    }
  }

  //When the striker lost the ball, he will search the ball in a triangle if he has already look around him
  state(triangleSearch)
  {
     transition
    {
      //We look to see if the striker is on the left. In this case it will move towards the decreasing y
      if (Geometry::isPointInsideRectangle2( Vector2f(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosLeftFieldBorder),
                                              Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosCenterGoal), 
                                              theRobotPose.translation))
      {
        goto moveToRight;
      }
      //We look to see if the striker is on the right. In this case it will move towards the increasing y
      else if (Geometry::isPointInsideRectangle2( Vector2f(theFieldDimensions.xPosOwnGoal, theFieldDimensions.yPosCenterGoal+1),
                                                  Vector2f(theFieldDimensions.xPosOpponentGoal, theFieldDimensions.yPosRightFieldBorder), 
                                                  theRobotPose.translation))
      {
        goto moveToLeft;
      }
      else
      {
        goto ballNotFound;
      }
    }
    action
    {
    }
  }

  state(moveToRight)
  {
    Vector2f targetPoint;
    targetPoint.x() = (theTeamBallModel.position).x() - walkDistance/2;
    targetPoint.y() = (theTeamBallModel.position).y() - walkDistance/2;
    transition
    {
      if( std::abs((theRobotPose.translation).x() - targetPoint.x()) < 5
          && std::abs((theRobotPose.translation).y() - targetPoint.y()) < 5)
      {
        goto moveToFrontFromRight;
      }
    }
    action
    {
      TrackBall();
      WalkToTarget( Pose2f(5.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * targetPoint).angle(),theRobotPose.inversePose * targetPoint), true);
    }
  }

  state(moveToFrontFromRight)
  {
    Vector2f targetPoint;
    targetPoint.x() = (theTeamBallModel.position).x() + walkDistance/2;
    targetPoint.y() = (theTeamBallModel.position).y();
    transition
    {
      if( std::abs((theRobotPose.translation).x() - targetPoint.x()) < 5
          && std::abs((theRobotPose.translation).y() - targetPoint.y()) < 5)
      {
        goto moveBackLeft;
      }
    }
    action
    {
      TrackBall();
      WalkToTarget( Pose2f(5.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * targetPoint).angle(),theRobotPose.inversePose * targetPoint), true);
    }
  }

  state(moveBackLeft)
  {
    Vector2f targetPoint;
    targetPoint.x() = (theTeamBallModel.position).x() - walkDistance/2;
    targetPoint.y() = (theTeamBallModel.position).y() + walkDistance/2;
    transition
    {
      if( std::abs((theRobotPose.translation).x() - targetPoint.x()) < 5
          && std::abs((theRobotPose.translation).y() - targetPoint.y()) < 5)
      {
        goto ballNotFound;
      }
    }
    action
    {
      TrackBall();
      WalkToTarget( Pose2f(5.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * targetPoint).angle(),theRobotPose.inversePose * targetPoint), true);
    }
  }

  state(moveToLeft)
  {
    Vector2f targetPoint;
    targetPoint.x() = (theTeamBallModel.position).x() - walkDistance/2;
    targetPoint.y() = (theTeamBallModel.position).y() + walkDistance/2;
    transition
    {
      if( std::abs((theRobotPose.translation).x() - targetPoint.x()) < 5
          && std::abs((theRobotPose.translation).y() - targetPoint.y()) < 5)
      {
        goto moveToFrontFromLeft;
      }
    }
    action
    {
      TrackBall();
      WalkToTarget( Pose2f(5.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * targetPoint).angle(),theRobotPose.inversePose * targetPoint), true);
    }
  }

  state(moveToFrontFromLeft)
  {
    Vector2f targetPoint;
    targetPoint.x() = (theTeamBallModel.position).x() + walkDistance/2;
    targetPoint.y() = (theTeamBallModel.position).y();
    transition
    {
      if( std::abs((theRobotPose.translation).x() - targetPoint.x()) < 5
          && std::abs((theRobotPose.translation).y() - targetPoint.y()) < 5)
      {
        goto moveBackRight;
      }
    }
    action
    {
      WalkToTarget( Pose2f(5.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * targetPoint).angle(),theRobotPose.inversePose * targetPoint), true);
    }
  }

  state(moveBackRight)
  {
    Vector2f targetPoint;
    targetPoint.x() = (theTeamBallModel.position).x() - walkDistance/2;
    targetPoint.y() = (theTeamBallModel.position).y() - walkDistance/2;
    transition
    {
      if( std::abs((theRobotPose.translation).x() - targetPoint.x()) < 5
          && std::abs((theRobotPose.translation).y() - targetPoint.y()) < 5)
      {
        goto ballNotFound;
      }
    }
    action
    {
      TrackBall();
      WalkToTarget( Pose2f(5.f, 100.f, 100.f), Pose2f((theRobotPose.inversePose * targetPoint).angle(),theRobotPose.inversePose * targetPoint), true);
    }
  }

  // If we are here, the ball hasn't been found, so the NAO striker should start again the triangle.
  aborted_state(ballNotFound)
  {
    transition
    {
      if(action_done)
        goto StartSearchTriangle;
    }
    action
    {
      SetHeadPanTilt(0, 0);
      
      if(theBehaviorStatus.role != Role::keeper)
      {
        MakeTurn(pi/2);
      }
    }
  }
}
