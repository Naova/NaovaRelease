option(DefensiveKickIn)
{
    initial_state(start)
    {
        transition
        {
            if(theTeamBallModel.isValid)
            {
                if (theLibCodeRelease.closerToTheBallDef)
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

    state(wall)
    {
        transition
        {
            if(theLibCodeRelease.timeSinceBallWasSeen > theBehaviorParameters.ballNotSeenTimeOut || !theLibCodeRelease.closerToTheBallDef) 
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

    state(playNormal)
    {
        action
        {
            HandleTeamTactic();
        }
    }
}