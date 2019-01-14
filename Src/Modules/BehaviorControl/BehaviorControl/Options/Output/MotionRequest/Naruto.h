/** Sets all members of the MotionRequest representation for simple standing */
option(Naruto)
{
	/** Set the motion request. */
	initial_state(setRequest)
	{
		transition
		{
		  if ( theArmMotionInfo.armMotion[Arms::left] == ArmMotionRequest::keyFrame)
			goto requestIsExecuted;
		}
			action
		{
			//theArmMotionRequest. = ArmKeyFrameRequest::back;
		  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
		theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
		}
	}

	/** The motion process has started executing the request. */
	target_state(requestIsExecuted)
	{
		transition
		{
		  if (theArmMotionInfo.armMotion[Arms::left] != ArmMotionRequest::keyFrame)
			goto setRequest;
		}
			action
		{
		 theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::keyFrame;
		theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::keyFrame;
		}
	}
}
