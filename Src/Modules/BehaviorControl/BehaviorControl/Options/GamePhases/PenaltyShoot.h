option(PenaltyShoot)
{
    initial_state(start)
    {
        action
        {
            if (theGameInfo.kickingTeam != Global::getSettings().teamNumber){
                DefensivePenaltyShoot();
            } else {
                OffensivePenaltyShoot();
            }
        }
    }
}