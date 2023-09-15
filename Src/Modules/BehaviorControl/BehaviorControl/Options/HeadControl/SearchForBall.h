option(SearchForBall)
{
    common_transition
    {
        // if(theTeamBallModel.isValid)
        //     goto ballFound;
    }

    initial_state(turn)
    {
        transition
        {
            if(action_done)
                goto ballFound;
        }
        action
        {
            TurnInPlace(0.5f, -160_deg);
        }
    }

    target_state(ballFound)
    {
        Stand();
    }
}