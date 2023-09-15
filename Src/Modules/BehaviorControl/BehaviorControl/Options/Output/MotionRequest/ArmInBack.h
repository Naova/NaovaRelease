/** State to put arm in the back
*/
option(ArmInBack)
{
	/** Set the motion request. */
	initial_state(setRequest)
	{
		transition
		{
			float obstacleAngle;
			float obstacleDistance;
			bool found_obstacle_center = false;
			bool found_obstacle_left = false;
			bool found_obstacle_right = false;
			for (const Obstacle &obstacle : theObstacleModel.obstacles)
			{
				Vector2f point = obstacle.center;
				obstacleDistance = point.norm();
				obstacleAngle = point.angle();
				if (obstacleDistance < theBehaviorParameters.maxObstacleDistanceArmsInBack)
				{
					if (obstacleAngle > theBehaviorParameters.leftAngleArmsInBack)
					{
						found_obstacle_left = true;
					}
					else if (obstacleAngle < -theBehaviorParameters.leftAngleArmsInBack)
					{
						found_obstacle_right = true;
					}
					else
					{
						found_obstacle_center = true;
					}
				}
			}
			if (theFallDownState.state != theFallDownState.upright) {
				goto doNone;
			}
			if(found_obstacle_center || (found_obstacle_left && found_obstacle_right)) {
				goto armBackBoth;
			} else if(found_obstacle_left) {
				goto armBackBoth;
			} else if(found_obstacle_right) {
				goto armBackBoth;
			} else {
				goto doNone;
			}
		}
		action
		{}
	}

	/** The motion process has started executing the request. */
	target_state(armBackBoth)
	{
		transition
		{
			if (theArmMotionInfo.armMotion[Arms::left] == ArmMotionRequest::keyFrame && theFallDownState.state == theFallDownState.upright && (theLegMotionSelection.targetMotion == MotionRequest::walk || theLegMotionSelection.targetMotion == MotionRequest::kick))
				goto setRequest;
			else if (theFallDownState.state != theFallDownState.upright)
				goto doNone;
		}
		action
		{
			theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
			theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
		}
	}

	/** Put right arm in the back */
	target_state(armBackRight)
	{
		transition
		{
			if (theArmMotionInfo.armMotion[Arms::right] == ArmMotionRequest::keyFrame && theFallDownState.state == theFallDownState.upright && (theLegMotionSelection.targetMotion == MotionRequest::walk || theLegMotionSelection.targetMotion == MotionRequest::kick))
				goto setRequest;
			else if (theFallDownState.state != theFallDownState.upright)
				goto doNone;
		}
		action
		{
			theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
			//theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
		}
	}

	/** Put left arm in the back */
	target_state(armBackLeft)
	{
		transition
		{
			if (theArmMotionInfo.armMotion[Arms::left] == ArmMotionRequest::keyFrame && theFallDownState.state == theFallDownState.upright && (theLegMotionSelection.targetMotion == MotionRequest::walk || theLegMotionSelection.targetMotion == MotionRequest::kick))
				goto setRequest;
			else if (theFallDownState.state != theFallDownState.upright)
				goto doNone;
		}
		action
		{
			theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
			//theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;
		}
	}

	target_state(doNone)
	{
		transition
		{
			if (theFallDownState.state == theFallDownState.upright)
				goto setRequest;
		}
		action
		{
			theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
			theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;
		}
	}
}
