
/**
 * @file DynamicSupporterPositionningProvider.h
 *
 * This file declares a module that dynamically changes a supporter's positionning
 *
 * @author Jeremy Thu-Thon
 */

#pragma once

#include "Representations/BehaviorControl/DynamicSupporterPositionning.h"
#include "Representations/BehaviorControl/SupporterPositioning.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/BehaviorControl/CurrentTactic.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"
#include "Tools/BehaviorControl/BehaviorUtilities.h"
#include "Tools/Math/Geometry.h"
#include "Tools/Module/Module.h"

MODULE(DynamicSupporterPositionningProvider,
{,
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotDimensions),
  REQUIRES(CurrentTactic),
  REQUIRES(GameInfo),
  REQUIRES(ExtendedGameInfo),
  REQUIRES(TeamData),
  REQUIRES(RobotInfo),
  REQUIRES(FieldBall),
  REQUIRES(RobotPose),
  REQUIRES(ObstacleModel),
  USES(SupporterPositioning),
  PROVIDES(DynamicSupporterPositionning),
});

class DynamicSupporterPositionningProvider : public DynamicSupporterPositionningProviderBase
{
  typedef std::vector<Vector2f> BaseArea;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theDynamicSupporterPositionning The representation updated.
   */
  void update(DynamicSupporterPositionning& theDynamicSupporterPositionning) override;

  private:

    std::vector<Vector2f> listePoints;
    Vector2f basePose ;
   
    const float stepMultiplier {0.75};
    const std::vector<float> steps{1000, 500, 250};

    int minX{std::numeric_limits<int>::max()};
    int minY{std::numeric_limits<int>::max()};
    int maxX{std::numeric_limits<int>::lowest()};
    int maxY{std::numeric_limits<int>::lowest()};

    // Fonction pour trouver les limites du polygone
    void findBounds(std::vector<Vector2f> area);

    std::vector<Vector2f> scanArea(float step, std::vector<Vector2f> area);
};
