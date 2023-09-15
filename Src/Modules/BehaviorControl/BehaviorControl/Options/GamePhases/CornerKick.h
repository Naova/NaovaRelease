option(CornerKick)
{
    initial_state(start)
    {
        transition
        {
            if (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
                goto offensiveCornerKick;
            }
            else if ((theRobotPose.inversePose*theTeamBallModel.position).norm() < theBehaviorParameters.safeDistance)
            {
                goto walkToSafeDistance;
            }
            else
            {
                goto defensiveCornerKick;
            }
        }    
    }

    state(offensiveCornerKick)
    {
        action
        {
            OffensiveCornerKick();
        }
    }

    state(walkToSafeDistance)
    {
        transition
        {
            if (action_done)
            {
                goto defensiveCornerKick;
            }
            
        }
        action
        {
            
            TrackBall();
            WalkToSafeDistance();
        }
    }

    state(defensiveCornerKick)
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
            DefensiveCornerKick();
        }
    }
}