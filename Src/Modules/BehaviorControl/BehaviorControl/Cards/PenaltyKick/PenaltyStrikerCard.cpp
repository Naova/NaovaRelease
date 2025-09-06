/**
 * @file PenaltyStrikerCard.cpp
 *
 * This file implements the behavior for the penalty striker
 *
 * @author Olivier St-Pierre
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Communication/GameInfo.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/BehaviorControl/BehaviorUtilities.h"

CARD(PenaltyStrikerCard,
{,
  CALLS(Activity),
  CALLS(PenaltyStrikerGoToBallAndKick),
  CALLS(WalkAtRelativeSpeed),
  CALLS(Stand),
  CALLS(LookAtBall),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(GameInfo),
  REQUIRES(OwnTeamInfo),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
});

class PenaltyStrikerCard : public PenaltyStrikerCardBase
{
    bool preconditions() const override
  {
    return (theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber && theGameInfo.setPlay == SET_PLAY_PENALTY_KICK  && theTeamBehaviorStatus.role.playsTheBall()) || (theGameInfo.gamePhase == GAME_PHASE_PENALTYSHOOT && theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber);
  }

  bool postconditions() const override
  {
    return !(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber && theGameInfo.setPlay == SET_PLAY_PENALTY_KICK  && theTeamBehaviorStatus.role.playsTheBall());
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::kickAtGoal);
    Vector2f target = BehaviorUtilities::bestScorePositionInGoal(theFieldBall.positionOnField, theObstacleModel.obstacles, theRobotPose);
    Pose2f kickPose = Pose2f((theRobotPose.inversePose*target).angle(), theFieldBall.endPositionRelative);
    thePenaltyStrikerGoToBallAndKickSkill(kickPose, KickInfo::KickType::forwardFastRight, 0.5);
  }

};

MAKE_CARD(PenaltyStrikerCard);