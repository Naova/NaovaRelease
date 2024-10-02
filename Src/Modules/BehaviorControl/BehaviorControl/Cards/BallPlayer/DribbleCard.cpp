/**
 * @file DribbleCard.cpp
 *
 * This file implements the dribble behavior
 *
 * @author Elias et Marc-Olivier
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include "Representations/BehaviorControl/FieldRating.h"
#include "Tools/Debugging/DebugDrawings.h"

CARD(DribbleCard,
{,
  CALLS(Activity),
  CALLS(GoToBallAndDribble),
  CALLS(WalkAtRelativeSpeed),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(BallPlayerStrategy),
  REQUIRES(FieldRating),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (float) (0.5f) dribbleSpeed,
    (int)(100) iterationSteps, /**< Num of iteration steps. */
    (float)(10.f) stepLength, /**< Iterate this much per step. */
    (float)(15.f) searchStepDrawWidth,
    (float)(5.f) searchStepDrawScale,
    (int)(10) searchStepDrawModulo,
  }),
});

class DribbleCard : public DribbleCardBase
{
  void preProcess() override
  {
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:direction", "drawingOnField");
    DECLARE_DEBUG_DRAWING("skill:DribbleToGoal:step", "drawingOnField");
  }

  bool preconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::dribble;
  }

  bool postconditions() const override
  {
    return theBallPlayerStrategy.currentStrategy != BallPlayerStrategy::Strategy::dribble;
  }

  option
  {
    theActivitySkill(BehaviorStatus::dribble);

    initial_state(goToBallAndDribble)
    {
      action
      {
        Angle lastDribbleAngle = 0_deg;
        PotentialValue potentialValue;
        Vector2f position = theFieldBall.endPositionOnField;

        const Vector2f startPosition = position - position.normalized(0.001f); // To ensure we are not on the field line
        position += Vector2f::polar(100.f, lastDribbleAngle);
    
        for(int i = 0; i < iterationSteps; i++)
        {
          potentialValue = theFieldRating.potentialFieldOnly(position.x(), position.y(), true);
          theFieldRating.getObstaclePotential(potentialValue, position.x(), position.y(), true);
          theFieldRating.potentialWithRobotFacingDirection(potentialValue, position.x(), position.y(), true);
          const float directionNorm = potentialValue.direction.norm();
          if(directionNorm == 0.f)
            break;
          Vector2f direction = potentialValue.direction / directionNorm;
          direction *= stepLength;

          COMPLEX_DRAWING("skill:DribbleToGoal:step")
          {
            if(i % searchStepDrawModulo == 0 && i != 0) // skip first
              ARROW("skill:DribbleToGoal:step", position.x(), position.y(), position.x() + searchStepDrawScale * direction.x(), position.y() + searchStepDrawScale * direction.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::black);
          }


          position += direction;
          if(position.x() > theFieldDimensions.xPosOpponentGroundLine && position.y() < theFieldDimensions.yPosLeftGoal && position.y() > theFieldDimensions.yPosRightGoal)
            break;
        }

        Angle dribbleAngleInField = (position - startPosition).angle();

        COMPLEX_DRAWING("skill:DribbleToGoal:direction")
        {
          const Vector2f pos2 = theFieldBall.endPositionOnField + Vector2f::polar(300.f, dribbleAngleInField);
          ARROW("skill:DribbleToGoal:direction", theFieldBall.endPositionOnField.x(), theFieldBall.endPositionOnField.y(), pos2.x(), pos2.y(), searchStepDrawWidth, Drawings::solidPen, ColorRGBA::cyan);
        }

        const Angle dribbleAngle = Angle::normalize(dribbleAngleInField - theRobotPose.rotation);
        lastDribbleAngle = dribbleAngle;

        theGoToBallAndDribbleSkill(dribbleAngle, false, dribbleSpeed, false, false);
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f)).angle();
  }
};

MAKE_CARD(DribbleCard);
