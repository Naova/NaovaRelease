/**
 * @file ReadyCard.cpp
 *
 * This file implements the behavior for the ready.
 *
 * @author Olivier St-Pierre
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/BehaviorControl/SupporterPositioning.h"

CARD(ReadyCard,
{,
  CALLS(Activity),
  CALLS(WalkToKickoffPose),
  REQUIRES(GameInfo),
  REQUIRES(SupporterPositioning),
});

class ReadyCard : public ReadyCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.state == STATE_READY;
  }

  bool postconditions() const override
  {
    return theGameInfo.state != STATE_READY;;
  }

  void execute() override
  {
    theActivitySkill(BehaviorStatus::ready);
    Pose2f targetAbsolute = theSupporterPositioning.basePose;
    theWalkToKickoffPoseSkill(targetAbsolute);
  }
};

MAKE_CARD(ReadyCard);
