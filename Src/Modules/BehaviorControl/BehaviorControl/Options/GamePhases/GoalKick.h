option(GoalKick)
{
    initial_state(start)
    {
        transition
        {
            if (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
                goto offensiveGoalKick;
            }
            else if ((theRobotPose.inversePose*theTeamBallModel.position).norm() < theBehaviorParameters.safeDistance)
            {
                goto walkToSafeDistance;
            }
            else
            {
                goto defensiveGoalKick;
            }
        }      
    }

    state(offensiveGoalKick)
    {
        action
        {
            OffensiveGoalKick();
        }
    }

    state(walkToSafeDistance)
    {
        transition
        {
            if (action_done)
            {
                goto defensiveGoalKick;
            }
            
        }
        action
        {
            
            TrackBall();
            WalkToSafeDistance();
        }
    }

    state(defensiveGoalKick)
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
            DefensiveGoalKick();
        }
    }
}