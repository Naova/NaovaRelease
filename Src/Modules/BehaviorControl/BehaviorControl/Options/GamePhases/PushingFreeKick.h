option(PushingFreeKick)
{
    initial_state(start)
    {
        transition
        {
            if (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
                goto offensivePushingFreeKick;
            }
            else if ((theRobotPose.inversePose*theTeamBallModel.position).norm() < theBehaviorParameters.safeDistance)
            {
                goto walkToSafeDistance;
            }
            else
            {
                goto defensivePushingFreeKick;
            }
        }       
    }

    state(offensivePushingFreeKick)
    {
        action
        {
            OffensivePushingFreeKick();
        }
    }

    state(walkToSafeDistance)
    {
        transition
        {
            if (action_done)
            {
                goto defensivePushingFreeKick;
            }
            
        }
        action
        {
            
            TrackBall();
            WalkToSafeDistance();
        }
    }

    state(defensivePushingFreeKick)
    {
        transition
        {
            if ((theRobotPose.inversePose*theTeamBallModel.position).norm() < theBehaviorParameters.safeDistance)
            {
                goto walkToSafeDistance;
            }
        }
        action
        {
            DefensivePushingFreeKick();
        }
    }
}