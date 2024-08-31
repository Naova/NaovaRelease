/**
 * @file SmartKick.cpp
 *
 * This file implements an implementation of the SmartKick skill.
 *
 * @author Elias
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Skill/Skill.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(SmartKickImpl,
{,
  IMPLEMENTS(SmartKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(GoToBallAndKick),
  CALLS(WalkToBallAndKick),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(1500.f) walkingKickDistance,
  }),
});

class SmartKickImpl : public SmartKickImplBase
{
    option(SmartKick)
    {
        initial_state(start)
        {
            transition
            {
                if (calcSineOfAngle(p.kickTarget) < 0.f) 
                    goto goToBallAndKickLeft;
                else
                    goto goToBallAndKickRight;
            }

            action
            {
                theLookForwardSkill();
                theStandSkill();
            }
        }

        state(goToBallAndKickLeft)
        {
            transition
            {
                if (calcSineOfAngle(p.kickTarget) > 0.1f)
                    goto goToBallAndKickRight;
            }

            action
            {
                theGoToBallAndKickSkill(calcAngleToTarget(p.kickTarget), getKickType(p.kickTarget, true));
            }
        }

        state(goToBallAndKickRight)
        {
            transition
            {
                if (calcSineOfAngle(p.kickTarget) < -0.1f)
                    goto goToBallAndKickLeft;
            }

            action
            {
                theGoToBallAndKickSkill(calcAngleToTarget(p.kickTarget), getKickType(p.kickTarget, false));
            }
        }
    }

    float calcSineOfAngle(Vector2f kickTarget) const
    {
        Vector2f robotToBall = theFieldBall.positionOnField - theRobotPose.translation;
        Vector2f ballToTarget = kickTarget - theFieldBall.positionOnField;

        return (robotToBall.x() * ballToTarget.y() - robotToBall.y() * ballToTarget.x());
    }

    Angle calcAngleToTarget(Vector2f kickTarget) const
    {
        return (theRobotPose.inversePose * kickTarget).angle();
    }

    KickInfo::KickType getKickType(Vector2f kickTarget, bool left)
    {
        KickInfo::KickType type;
        float distance = (kickTarget - theFieldBall.positionOnField).norm();

        if (distance >= walkingKickDistance)
            type = KickInfo::forwardFastRight;
        else
            type = KickInfo::walkForwardsRight;

        return left ? static_cast<KickInfo::KickType>(static_cast<int>(type) + 1) : type;
    }
};

MAKE_SKILL_IMPLEMENTATION(SmartKickImpl);