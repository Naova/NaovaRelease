/**
 * @file PenaltyKeeperCard.cpp
 *
 * This file implements the behavior for the penalty keeper
 *
 * @author Olivier St-Pierre
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Tools/BehaviorControl/Interception.h"

CARD(PenaltyKeeperCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndKick),
  CALLS(Stand),
  CALLS(InterceptBall),
  REQUIRES(OwnTeamInfo),
  REQUIRES(GameInfo),
  REQUIRES(TeamBehaviorStatus),
});

class PenaltyKeeperCard : public PenaltyKeeperCardBase
{
    bool preconditions() const override
  {
    return (theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber && theGameInfo.setPlay == SET_PLAY_PENALTY_KICK && theTeamBehaviorStatus.role.isGoalkeeper()) || (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber);
  }

  bool postconditions() const override
  {
    return !(theGameInfo.kickingTeam != theOwnTeamInfo.teamNumber && theGameInfo.setPlay == SET_PLAY_PENALTY_KICK && theTeamBehaviorStatus.role.isGoalkeeper());
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::interceptBall);
    theInterceptBallSkill(bit(Interception::walk) | bit(Interception::jumpLeft) | bit(Interception::jumpRight) | bit(Interception::stand), true, true);
  }
};

MAKE_CARD(PenaltyKeeperCard);