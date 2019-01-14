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
#include "Representations/Communication/TeamData.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"

MODULE(LibCodeReleaseProvider,
{,
  REQUIRES(BallModel),
  REQUIRES(TeamBallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  USES(BehaviorStatus),
  PROVIDES(LibCodeRelease),
});

class LibCodeReleaseProvider : public LibCodeReleaseProviderBase
{
  void update(LibCodeRelease& libCodeRelease);

  private:

    void countRoles(LibCodeRelease& libCodeRelease);
    void getDesiredPos(LibCodeRelease& libCodeRelease);
    int getMyNumberDefender();
    Vector2f findStrikerPos();
    Vector2f findSupportPos();
    bool isCloserToTheBall();
    bool isBallInPenaltyZone();
    bool isBallInZone(Vector2f pointA,Vector2f pointB);

    Vector2f desiredPos;
    double distanceToBall;
    int nbOfDefender;
    int nbOfKeeper;
    int nbOfStriker;
    int nbOfSupporter;


};
