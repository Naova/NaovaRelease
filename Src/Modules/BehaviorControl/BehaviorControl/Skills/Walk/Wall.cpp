/**
 * @file Wall.cpp
 *
 * This file implements an implementation of the wall skill.
 *
 * @author Marc-Olivier
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"

SKILL_IMPLEMENTATION(WallImpl,
{,
  IMPLEMENTS(Wall),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  CALLS(WalkToPoint),
  DEFINES_PARAMETERS(
  {,
    (float)(950.f) updatedWallDistance, /** the updated parameter of theWallSkill Parameter */
  }),
});

class WallImpl : public WallImplBase
{
    void preProcess() override {}

    void preProcess(const Wall& p) override
    {
        if (p.wallDistance > updatedWallDistance)
            updatedWallDistance = p.wallDistance;
    }

    void execute(const Wall&) override
    {
        Pose2f wallPosition = calculateGoalWallPosition(updatedWallDistance);
        theWalkToPointSkill(wallPosition);
    }

    Pose2f calculateGoalWallPosition(float wallDistance)
    {
        Vector2f goalPosition = Vector2f(0,0);
        Vector2f ballPosition = theFieldBall.teamPositionOnField;

        if (ballPosition.x() < theFieldDimensions.xPosOwnGoalArea)
        {
            goalPosition = Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosCenterGoal);
        }
        else
        {
            if (ballPosition.y() > theFieldDimensions.yPosCenterGoal)
            {
                goalPosition = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoal / 3.f);
            }
            else
            {
                goalPosition = Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoal / 3.f);
            }
        }

        Vector2f distanceFromBall = (goalPosition - ballPosition).normalized();
        Vector2f wallPosition = theRobotPose.inversePose*(ballPosition + distanceFromBall*wallDistance);

        return Pose2f((theRobotPose.inversePose*ballPosition).angle(), wallPosition);
    }
};

MAKE_SKILL_IMPLEMENTATION(WallImpl);
