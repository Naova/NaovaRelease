/**
 * @file TurnInPlace.cpp
 *
 * This file implements an implementation of the TurnInPlace skill
 * This skill makes the robot turn the amount that we put in the angle parameter.
 * The turn can be greater than 2pi to turn various times.
 *
 * @author Elias
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Math/Angle.h"
#include <cmath>
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(TurnInPlaceImpl,
{,
  IMPLEMENTS(TurnInPlace),
	CALLS(WalkAtRelativeSpeed),
	CALLS(Stand),
  REQUIRES(OdometryData),
	REQUIRES(FrameInfo),
	DEFINES_PARAMETERS(
  {,
    (unsigned int)(100) updateInterval, /**< The interval at which we will update the rotation value */
  }),
});

class TurnInPlaceImpl : public TurnInPlaceImplBase
{
	float lastRotation;
	float currentRotation;
	float rotationChange;
	int lastRotationUpdateTime;
	float accumulatedRotation;

  option(TurnInPlace)
  {
		float turnDirection = p.angle > 0 ? 0.8f : -0.8f;

    initial_state(start)
    {
      transition
      {
				if(state_time > 500)
					goto turn;
      }
			action
			{
				accumulatedRotation = 0.f;
				lastRotation = theOdometryData.rotation;
				lastRotationUpdateTime = theFrameInfo.time;
				
				theStandSkill();
			}
    }

		state(turn)
		{
			transition
			{
				if(theFrameInfo.time != 0 && theFrameInfo.time - lastRotationUpdateTime > updateInterval)
      	{
					lastRotationUpdateTime = theFrameInfo.time;
					currentRotation = theOdometryData.rotation;
					rotationChange = (float)Angle::normalize(currentRotation - lastRotation);
					accumulatedRotation = accumulatedRotation + std::abs(rotationChange);
					lastRotation = currentRotation;

					if(accumulatedRotation >= std::abs(p.angle))
					{
						goto finished;
					}
				}
			}
			action
			{
				theWalkAtRelativeSpeedSkill(Pose2f(turnDirection));
			}
		}

    target_state(finished)
		{
			action
			{
				theStandSkill();
			}
		}
  }
};

MAKE_SKILL_IMPLEMENTATION(TurnInPlaceImpl);
