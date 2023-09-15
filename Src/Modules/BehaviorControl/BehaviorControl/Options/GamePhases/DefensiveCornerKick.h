option(DefensiveCornerKick)
{
    initial_state(start)
    {
        transition
        {
            goto turnAndBack;
        }
    }

    state(turnAndBack)
    {
        transition
        {
            if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
            {
                goto backOff;
            }
        }
        action
        {
            LookForward();
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
        }
    }
    
    state(backOff)
    {
        transition
        {
            if((theRobotPose.inversePose * theTeamBallModel.position).norm() > 760.f)
            {
                goto standUp;
            }
        }
        action
        {
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), -200.f, 0.f));
        }
    }
    
    state(standUp)
    {
        action
        {
            theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
            theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;
            LookForward();
            Stand();
        }
    }
}