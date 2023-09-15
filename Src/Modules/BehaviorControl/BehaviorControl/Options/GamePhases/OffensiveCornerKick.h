option(OffensiveCornerKick)
{
    initial_state(start)
    {
        transition
        {
            if(theLibCodeRelease.closerToTheBall)
            {
                goto turnAndKick;
            }
            else
            {
                goto turnAndBack;
            }
        }
    }

    state(turnAndKick)
    {
        transition
        {
            if(std::abs((theRobotPose.inversePose * theTeamBallModel.position).angle()) < 5_deg)
            {
                goto walkToBall;
            }
        }
        action
        {
            LookForward();
            WalkToTarget(Pose2f(50.f, 50.f, 50.f), Pose2f((theRobotPose.inversePose * theTeamBallModel.position).angle(), 0.f, 0.f));
        }
    }

    state(walkToBall)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() < 500.f)
            {
                goto alignToGoal;
            }
        }
        action
        {
            LookForward();
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), theRobotPose.inversePose * theTeamBallModel.position);
        }
    }

    state(alignToGoal)
    {
        transition
        {
            if(std::abs(theLibCodeRelease.angleToOppGoal) < 10_deg && std::abs(theBallModel.estimate.position.y()) < 100.f)
            {
                goto alignBehindBall;
            }
            if(theBallModel.estimate.position.norm() > 600.f)
            {
                goto walkToBall;
            }
        }
        action
        {
            LookForward();
            WalkToTarget(Pose2f(100.f, 100.f, 100.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 400.f, theBallModel.estimate.position.y()));
        }
    }

    state(alignBehindBall)
    {
        transition
        {
            if(theLibCodeRelease.between(theBallModel.estimate.position.y(), 20.f, 50.f)
                && theLibCodeRelease.between(theBallModel.estimate.position.x(), 140.f, 170.f)
                && std::abs(theLibCodeRelease.angleToOppGoal) < 2_deg)
            {
                goto kick;
            }
            if(theBallModel.estimate.position.norm() > 600.f)
            {
                goto walkToBall;
            }
        }
        action
        {
            LookForward();
            WalkToTarget(Pose2f(80.f, 80.f, 80.f), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 150.f, theBallModel.estimate.position.y() - 30.f));
        }
    }

    state(kick)
    {
        transition
        {
            if(theBallModel.estimate.position.norm() > 600.f || state_time > 3000 || (state_time > 10 && action_done))
            {
                goto walkToBall;
            }
        }
        action
        {
            LookForward();
            InWalkKick(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(theLibCodeRelease.angleToOppGoal, theBallModel.estimate.position.x() - 160.f, theBallModel.estimate.position.y() - 55.f));
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