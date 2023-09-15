/**
 * @file LibCodeReleaseProvider.h
 */

#include "Tools/Module/Module.h"
#include "Representations/BehaviorControl/Libraries/RoleChanges.h"
#include "Representations/BehaviorControl/Libraries/LibCodeRelease.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Tools/Role.h"

MODULE(RoleChangesProvider,
{,
  REQUIRES(TeamBallModel),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(RobotInfo),
  REQUIRES(LibCodeRelease),
  REQUIRES(FrameInfo),
  USES(BehaviorStatus),
  PROVIDES(RoleChanges),
  DEFINES_PARAMETERS(
  {,
    (int)(500) roleChangeDelay,
    (bool) (true) isActive,
  }),
});

class RoleChangesProvider : public RoleChangesProviderBase
{
  void update(RoleChanges& roleChanges);

  private:

    void changeRole(RoleChanges& roleChanges, Role::RoleType roleType);
    void changeStriker(RoleChanges& roleChanges);
    void changeSupporter(RoleChanges& roleChanges);
    void changeDefender(RoleChanges& roleChanges, Role::RoleType roleType);
    bool needToChangeStriker();
    Role::RoleType getMissingRole();
    bool isCloserToOwnGoal(Role::RoleType roleType);
    // Quand on fera un changement de rôle qui regarde si les defenseurs sont plus d'un bord que de l'autre
    // bool isFurtherSideDefender(Role::RoleType roleType);

    double distanceToOwnGoal;
};
