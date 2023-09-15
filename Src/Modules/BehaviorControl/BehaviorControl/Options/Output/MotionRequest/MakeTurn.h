// make the Nao turn on itself
// If given a positive turnAngle, the robot will rotate toward the left
// If given a negative turnAngle, the robot will rotate toward the right

// TurnAngle is calculated in Radian (ex: pi = 180 degree, pi/2 = 90 degree, etc...)
option(MakeTurn, (Angle) turnAngle)
{
    // destAngle is where the Nao is supposed to end up
    static Angle destAngle = NULL;

    // currentAngle is where the NAO was at the beginning of the walkTotargetRequest
    static Angle currentAngle = NULL;

    initial_state(startTurn)
    {
        transition
        {
            destAngle = NULL;
            if(turnAngle < 0)
                goto setRequestTurnRight;
            else 
                goto setRequestTurnLeft;
        }
    }

    // Create the WalTotargetRequest for a right turn and calculate the coordinate of the destination
    state(setRequestTurnRight)
    {
        transition
        {
            // When the NAO starts walking, we calculate its destination
            if(theMotionInfo.motion == MotionRequest::walk) {
                destAngle = Angle(theRobotPose.rotation + turnAngle);
                currentAngle = Angle(theRobotPose.rotation);
                
                // if the calculated destination Angle is smaller than -180 (ex: -190), The Nao will not understand when to stop
                // We have to calculate the appropriate Angle of the destination
                if (destAngle <= -pi)
                {
                    destAngle = pi - ((destAngle + pi) * -1.0f);
                }
                goto TurnRightIsRequested;
            }
        }
        action
        {
            WalkToTarget(Pose2f(25.f, 25.f, 25.f), Pose2f(turnAngle, 0.f, 0.f));
        }
    }
    
    // Check the current coordinate of the NAO and goes to the target_state when the NAO gets to its destination
    state(TurnRightIsRequested)
    {
        transition
        {    
            if (destAngle >= 0) {
                if(theRobotPose.rotation <= destAngle && theRobotPose.rotation >= 0)
                    goto turnCompleted;
            } else {
                if(theRobotPose.rotation <= (currentAngle + theMotionRequest.walkRequest.target.rotation))
                    goto turnCompleted;
            }
            if (state_time >= turnAngle*3500){
                    goto turnCompleted;
            }
        }
    }

    // Create the WalTotargetRequest for a left turn and calculate the coordinate of the destination
    state(setRequestTurnLeft)
    {
        transition
        {
            if(theMotionInfo.motion == MotionRequest::walk) {
                destAngle = Angle(theRobotPose.rotation + turnAngle);
                currentAngle = Angle(theRobotPose.rotation);
                // if the calculated destination Angle is greater than 180 (ex: 190), The Nao will not understand when to stop
                // We have to calculate the appropriate Angle of the destination
                if (destAngle >= pi)
                {
                    destAngle = (-pi) + (destAngle - pi);
                }
                goto TurnLeftIsRequested;
            }
        }
        action
        {
            WalkToTarget(Pose2f(25.f, 25.f, 25.f), Pose2f(turnAngle, 0.f, 0.f));
        }
    }

    // Check the current coordinate of the NAO and goes to the target_state when the NAO gets to its destination
    state(TurnLeftIsRequested)
    {
        transition
        {    
            if (destAngle <= 0) {
                if(theRobotPose.rotation >= destAngle && theRobotPose.rotation <= 0)
                    goto turnCompleted;
            } else {
                if(theRobotPose.rotation >= (currentAngle + theMotionRequest.walkRequest.target.rotation))
                    goto turnCompleted;
            }
            if (state_time >= turnAngle*3500){
                goto turnCompleted;
            }
        }
    }

    // When entering this state, the robot is finished turning
    target_state(turnCompleted)
    {
        action
        {
            theMotionRequest.motion = MotionRequest::stand;
        }
    }
}