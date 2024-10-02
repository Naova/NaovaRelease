/**
 * @file SearchBall.cpp
 *
 * This file implements implementations of the SearchBall skill.
 *
 * @author Elias
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Skill/CabslSkill.h"
#include "Representations/BehaviorControl/SupporterPositioning.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/Math/Geometry.h"
#include <vector>

SKILL_IMPLEMENTATION(SearchBallImpl,
{,
  IMPLEMENTS(SearchBall),
  REQUIRES(SupporterPositioning),
  REQUIRES(RobotPose),
  CALLS(WalkToPoint),
  CALLS(WalkAtRelativeSpeed),
  CALLS(TurnInPlace),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) turnSpeed,
    (float)(300.f) tolerance,
  }),
});

class SearchBallImpl : public SearchBallImplBase
{
  option(SearchBall)
  {
    Vector2f center;

    initial_state(turnOnce)
    {
      transition
      {
        if(theTurnInPlaceSkill.isDone())
          goto goToCenter;
      }
      action
      {
        theTurnInPlaceSkill();
      }
    }

    state(goToCenter)
    {
      center = theRobotPose.inversePose * getCenter(theSupporterPositioning.baseArea);

      transition
      {
        if (center.norm() < tolerance)
          goto turnInPlace;
      }
      action
      {
        theWalkToPointSkill(center);
      }
    }

    state(turnInPlace)
    {
      center = theRobotPose.inversePose * getCenter(theSupporterPositioning.baseArea);

      transition
      {
        if (center.norm() >= tolerance)
          goto goToCenter;
      }
      action
      {
        theWalkAtRelativeSpeedSkill(Pose2f(turnSpeed, 0.f, 0.f));
      }
    }
  }

  Vector2f getCenter(const std::vector<Vector2f>& vertices) {
    Vector2f centroid(0, 0);
    std::size_t vertexCount = vertices.size();

    for(const auto& vertex : vertices) {
        centroid += vertex;
    }

    centroid /= vertexCount;

    return centroid;
  }
};

MAKE_SKILL_IMPLEMENTATION(SearchBallImpl);
