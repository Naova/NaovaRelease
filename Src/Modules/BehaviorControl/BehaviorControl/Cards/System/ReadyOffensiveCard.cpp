/**
 * @file ReadyOffensiveCard.cpp
 *
 * This file implements the behavior for the ready when we have the kickoff.
 *
 * @author Olivier St-Pierre
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/BehaviorControl/SupporterPositioning.h"

CARD(ReadyOffensiveCard,
{,
  CALLS(Activity),
  CALLS(WalkToKickoffPose),
  CALLS(Say),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotDimensions),
  REQUIRES(OwnTeamInfo),
  REQUIRES(GameInfo),
  REQUIRES(RobotPose),
  REQUIRES(SupporterPositioning),
});

class ReadyOffensiveCard : public ReadyOffensiveCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::positionForKickOff);
    Pose2f targetAbsolute = theSupporterPositioning.basePose;
    theWalkToKickoffPoseSkill(targetAbsolute);
  }
};

MAKE_CARD(ReadyOffensiveCard);
