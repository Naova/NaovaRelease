option(PlayingState)
{
    initial_state(playing)
    {
        transition
        {
            if (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT)
                goto penaltyShoot;
            if (theGameInfo.setPlay == SET_PLAY_NONE)
                goto teamTactic;
            else if (theGameInfo.setPlay == SET_PLAY_GOAL_KICK)
                goto goalKick;
            else if (theGameInfo.setPlay == SET_PLAY_PUSHING_FREE_KICK)
                goto pushingFreeKick;
            else if (theGameInfo.setPlay == SET_PLAY_CORNER_KICK)
                goto cornerKick;
            else if (theGameInfo.setPlay == SET_PLAY_KICK_IN)
                goto kickIn;
        }
    }

    state(penaltyShoot)
    {
        transition
        {
            if (theGameInfo.setPlay == SET_PLAY_NONE)
                goto playing;
        }
        action
        {
            PenaltyShoot();
        }
    }

    state(goalKick)
    {
        transition
        {
            if (theGameInfo.setPlay == SET_PLAY_NONE)
                goto playing;
        }
        action
        {
            GoalKick();
        }
    }

    state(pushingFreeKick)
    {
        transition
        {
            if (theGameInfo.setPlay == SET_PLAY_NONE)
                goto playing;
        }
        action
        {
            PushingFreeKick();
        }
    }

    state(cornerKick)
    {
        transition
        {
            if (theGameInfo.setPlay == SET_PLAY_NONE)
                goto playing;
        }
        action
        {
            CornerKick();
        }
    }

    state(kickIn)
    {
        transition
        {
            if (theGameInfo.setPlay == SET_PLAY_NONE)
                goto playing;
        }
        action
        {
            KickIn();
        }
    }

    state(teamTactic)
    {
        transition
        {
            if(theGameInfo.setPlay != SET_PLAY_NONE)
                goto playing;
        }
        action
        {
            HandleTeamTactic();
        }
    }
}