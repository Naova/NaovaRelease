option(DefensiveGoalKick)
{
    initial_state(start)
    {
        transition
        {
            if(theTeamBallModel.isValid)
            {
                if (theLibCodeRelease.closerToTheBall)
                {
                    goto wall;
                }
                else
                {
                    goto playNormal;
                }
            }
            else
            {
                goto SearchBall;
            }
        }
        action
        {
            TrackBall();
            Stand();
        }
    } 

    state(SearchBall)
    {
        transition
        {
            if (theTeamBallModel.isValid)
            {
                goto start;
            }
        }
        action
        {
            SearchBall();
        }
    }

    state(playNormal)
    {
        action
        {
            HandleTeamTactic();
        }
    }

    state(wall)
    {
        transition
        {           
            if(!theTeamBallModel.isValid || !theLibCodeRelease.closerToTheBall)
            {
                goto start;           
            } 
        }

        action
        {
            TrackBall(); 
            Wall(); 
        }
    }
}