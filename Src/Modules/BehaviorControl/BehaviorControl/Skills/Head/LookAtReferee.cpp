/**
 * @file LookAtReferee.cpp
 *
 * This file implements an implementation of the LookAtReferee skill.
 * Looks at the referee position on field and enables the referee detection modules.
 *
 * @author wilht
 */
#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Perception/RefereePercept/OptionalImageRequest.h"

SKILL_IMPLEMENTATION(LookAtRefereeImpl,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(GameInfo),
  CALLS(LookAtAngles),
  CALLS(LookAtPoint),
  MODIFIES(OptionalImageRequest),
  IMPLEMENTS(LookAtReferee),
});

class LookAtRefereeImpl : public LookAtRefereeImplBase
{
  const float lookAtHeight = 100.f;
  const Angle maxBearing = 45_deg;

  option(LookAtReferee)
  {
    const Vector2f refereeOnField = theFieldDimensions.refereeOnField(theGameInfo.leftHandTeam);
    const Vector2f refereeOffset = theRobotPose.inversePose * refereeOnField;
    const auto hasRefereeInSight = [&] { return std::abs(refereeOffset.angle()) <= maxBearing; };

    common_transition
    {
      if(theGameInfo.state != STATE_STANDBY || !hasRefereeInSight())
        goto inactive;
    }

    initial_state(inactive)
    {
      transition
      {
        if(theGameInfo.state == STATE_STANDBY && hasRefereeInSight())
          goto lookAtReferee;
      }
      action 
      {
        theLookAtAnglesSkill(0.f, 0.f, 150_deg);
      }
    }

    state(lookAtReferee)
    {
      action
      {
        theLookAtPointSkill(Vector3f(refereeOffset.x(), refereeOffset.y(), lookAtHeight),
                            HeadMotionRequest::upperCamera);
        theOptionalImageRequest.sendImage = true; // Enables referee detection modules
      }
    }
  }
};

MAKE_SKILL_IMPLEMENTATION(LookAtRefereeImpl);
