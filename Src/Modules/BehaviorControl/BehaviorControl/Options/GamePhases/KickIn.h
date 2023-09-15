option(KickIn)
{
    initial_state(start)
    {
        transition
        {
            if (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
                goto offensiveKickIn;
            }
            else if ((theRobotPose.inversePose*theTeamBallModel.position).norm() < theBehaviorParameters.safeDistance)
            {
                goto walkToSafeDistance;
            }
            else
            {
                goto defensiveKickIn;
            }
        }      
    }

    state(offensiveKickIn)
    {
        action
        {
            OffensiveKickIn();
        }
    }

    state(walkToSafeDistance)
    {
        transition
        {
            if (action_done)
            {
                goto defensiveKickIn;
            }
            
        }
        action
        {
            
            TrackBall();
            WalkToSafeDistance();
        }
    }

    state(defensiveKickIn)
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
            DefensiveKickIn();
        }
    }
}