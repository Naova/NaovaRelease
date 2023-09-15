/**
 * @file LibCodeReleaseProvider.h
 */

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Tools/Role.h"

MODULE(LibCodeReleaseProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(BehaviorParameters),
  USES(BehaviorStatus),
  PROVIDES(LibCodeRelease),
});

class LibCodeReleaseProvider : public LibCodeReleaseProviderBase
{
  void update(LibCodeRelease& libCodeRelease);

  private:

    void countRoles(LibCodeRelease& libCodeRelease);
    void getDesiredPos(LibCodeRelease& libCodeRelease);
    void getDefenderDesiredPos(LibCodeRelease& libCodeRelease);
    int getMyNumberDefender();
    Vector2f findStrikerPos();
    Vector2f findSupportPos();
    bool isCloserToTheBall();
    bool isCloserToTheBallDef();
    bool specificTeammateIsCloserToBall(Role::RoleType teammateRole);
    bool isBallInGoalArea();
    bool isRobotInPenaltyArea();
    bool defenderIsCloserToBall(Role::RoleType roleType);
    bool isBallInPenaltyZone();
    bool isBallInZone(Vector2f pointA,Vector2f pointB);
    void getRole(LibCodeRelease& libCodeRelease);
    void getLastGameState(LibCodeRelease& libCodeRelease);

    Vector2f desiredPos;
    Vector2f defenderDesiredPos;
    double distanceToBall;
    int currentGameState = STATE_INITIAL;

};
