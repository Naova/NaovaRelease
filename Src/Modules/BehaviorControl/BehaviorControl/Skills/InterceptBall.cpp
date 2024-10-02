/**
 * @file InterceptBall.cpp
 *
 * This file implements an implementation of the InterceptBall and AfterInterceptBall skills.
 *
 * @author Arne Hasselbring
 */

#include "Platform/BHAssert.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"

SKILL_IMPLEMENTATION(InterceptBallImpl,
{,
  IMPLEMENTS(InterceptBall),
  IMPLEMENTS(AfterInterceptBall),
  CALLS(Annotation),
  CALLS(GetUpEngine),
  CALLS(KeyframeMotion),
  CALLS(LookAtBall),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(WalkToPoint),
  REQUIRES(FallDownState),
  REQUIRES(FieldBall),
  REQUIRES(MotionInfo),
  USES(MotionRequest),
  DEFINES_PARAMETERS(
  {,
    /* Note: These thresholds also exist in the behavior parameters, but have a slightly other purpose there. */
    (float)(80.f) genuflectStandRadius, /**< The range that is covered with a genuflect. */
    (float)(400.f) walkRadius, /**< The range that is covered by just walking. */
    (float)(1000.f) jumpRadius, /**< The range that is covered with a keeper jump. */
  }),
});

class InterceptBallImpl : public InterceptBallImplBase
{
  option(InterceptBall)
  {
    initial_state(chooseAction)
    {
      transition
      {
        ASSERT(p.interceptionMethods != 0);
        const float positionIntersectionYAxis = theFieldBall.intersectionPositionWithOwnYAxis.y();
        OUTPUT_TEXT("Y: "<< positionIntersectionYAxis);
        unsigned interceptionMethods = p.interceptionMethods;

        left = positionIntersectionYAxis > 0.f;

        if(positionIntersectionYAxis == 0)
          goto walkWithoutIntersect;

        if((interceptionMethods & bit(Interception::genuflectStand)) && (between<float>(positionIntersectionYAxis, -genuflectStandRadius, genuflectStandRadius) || interceptionMethods < (bit(Interception::genuflectStand) << 1)))
        {
          if(p.allowDive)
            goto genuflectStand;
          else
            goto audioGenuflect;
        }

        if((interceptionMethods & bit(Interception::walk)) && (between<float>(positionIntersectionYAxis, -walkRadius, walkRadius) || interceptionMethods < (bit(Interception::walk) << 1)))
          goto walk;
        
        if((interceptionMethods & bit(Interception::jumpLeft)) && (positionIntersectionYAxis < jumpRadius || interceptionMethods < (bit(Interception::jumpLeft) << 1)))
        {
          if(p.allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }

        if((interceptionMethods & bit(Interception::jumpRight)) && (positionIntersectionYAxis > -jumpRadius || interceptionMethods < (bit(Interception::jumpRight) << 1)))
        {
          if(p.allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }

        ASSERT(interceptionMethods & bit(Interception::stand));
        goto stand;
      }
    }

    state(stand)
    {
      transition
      {
        if(!(theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto targetStand;

        ASSERT(p.interceptionMethods & bit(Interception::stand));
      }
      action
      {
        theAnnotationSkill("Intercept Ball Stand!");
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    state(walkWithoutIntersect)
    {
      transition
      {
        const float positionIntersectionYAxis = theFieldBall.intersectionPositionWithOwnYAxis.y();
        if(positionIntersectionYAxis != 0)
        goto chooseAction;
      }
      action
      {
        theAnnotationSkill("No Intercept Ball Walk!");
        theLookAtBallSkill();
        theWalkToPointSkill(Pose2f(0.f, 0.f, theFieldBall.positionRelative.y()), 1.f, /* rough: */ false, /* disableObstacleAvoidance: */ false, /* disableAligning: */ true);
      }
    }

    state(walk)
    {
      transition
      {
        const float positionIntersectionYAxis = theFieldBall.intersectionPositionWithOwnYAxis.y();
        if(positionIntersectionYAxis > walkRadius + 50.f|| positionIntersectionYAxis < -walkRadius - 50.f)
        {
          if(p.allowDive)
            goto keeperSitJump;
          else
            goto audioJump;
        }
      }
      action
      {
        theAnnotationSkill("Intercept Ball Walk!");
        theLookAtBallSkill();
        theWalkToPointSkill(Pose2f(0.f, 0.f, theFieldBall.intersectionPositionWithOwnYAxis.y()), 1.f, /* rough: */ false, /* disableObstacleAvoidance: */ false, /* disableAligning: */ true);
      }
    }

    state(genuflectStand)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000 &&
           !(between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -250.f, 250.f) &&
             theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto getUp;
      }
      action
      {
        theAnnotationSkill(left ? "Genuflect Left!" : "Genuflect Right!");
        theLookForwardSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::genuflectStand, /* mirror: */ !left);
      }
    }

    state(keeperSitJump)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000)
          goto getUp;
      }
      action
      {
        theAnnotationSkill(left ? "Keeper Jump Left!" : "Keeper Jump Right!");
        theLookAtBallSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::keeperJumpLeft, /* mirror: */ !left);
      }
    }

    state(audioGenuflect)
    {
      transition
      {
        goto targetStand;
      }
      action
      {
        theSaySkill("Genuflect");
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    state(audioJump)
    {
      transition
      {
        goto targetStand;
      }
      action
      {
        theSaySkill(left ? "Jump left" : "Jump right");
        theLookAtBallSkill();
        theStandSkill();
      }
    }

    state(getUp)
    {
      transition
      {
        if(theGetUpEngineSkill.isDone())
          goto targetStand;
      }
      action
      {
        theLookForwardSkill();
        theGetUpEngineSkill();
      }
    }

    target_state(targetStand)
    {
      action
      {
        theLookAtBallSkill();
        theStandSkill();
      }
    }
  }

  option(AfterInterceptBall)
  {
    initial_state(initial)
    {
      transition
      {
        ASSERT(state_time == 0);
        ASSERT(theMotionRequest.motion == MotionRequest::keyframeMotion && theMotionRequest.keyframeMotionRequest.keyframeMotion >= KeyframeMotionRequest::firstNonGetUpAction);

        // It can be assumed that AfterInterceptBall and InterceptBall are not used simultaneously. Therefore, left is reused here.
        left = !theMotionRequest.keyframeMotionRequest.mirror;

        if(theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::genuflectStand)
          goto genuflectStand;
        else if(theMotionRequest.keyframeMotionRequest.keyframeMotion == KeyframeMotionRequest::keeperJumpLeft)
          goto keeperSitJump;

        FAIL("This skill must only be called when a diving motion has been set last.");
      }
    }

    state(genuflectStand)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000 &&
           !(between<float>(theFieldBall.intersectionPositionWithOwnYAxis.y(), -250.f, 250.f) &&
             theFieldBall.ballWasSeen(300) &&
             between<float>(theFieldBall.timeUntilIntersectsOwnYAxis, 0.1f, 3.5f)))
          goto getUp;
      }
      action
      {
        theLookForwardSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::genuflectStand, /* mirror: */ !left);
      }
    }

    state(keeperSitJump)
    {
      transition
      {
        if(p.allowGetUp && state_time > 2000)
          goto getUp;
      }
      action
      {
        theLookAtBallSkill();
        theKeyframeMotionSkill(KeyframeMotionRequest::keeperJumpLeft, /* mirror: */ !left);
      }
    }

    state(getUp)
    {
      transition
      {
        if(theGetUpEngineSkill.isDone())
          goto getUpTarget;
      }
      action
      {
        theLookForwardSkill();
        theGetUpEngineSkill();
      }
    }

    target_state(getUpTarget)
    {
      action
      {
        theLookForwardSkill();
        theGetUpEngineSkill();
      }
    }
  }

  bool left; /**< Whether the interception is left of the robot (right otherwise). */
};

MAKE_SKILL_IMPLEMENTATION(InterceptBallImpl);