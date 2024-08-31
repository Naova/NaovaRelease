/**
 * @file SupporterPositioningProvider.h
 *
 * This file declares a module that provides the SupporterPositioning representation
 *
 * @author Olivier St-Pierre
 * @author Jérémie Thu-Thon
 */

#pragma once

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
#include "Representations/Infrastructure/ExtendedGameInfo.h"

#include "Tools/Module/Module.h"

#define SUB_NUMBER 6

MODULE(SupporterPositioningProvider,
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
  PROVIDES(SupporterPositioning),
});

class SupporterPositioningProvider : public SupporterPositioningProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theSupporterPositioning The representation updated.
   */
  void update(SupporterPositioning& theSupporterPositioning) override;

  private:
    struct Agent {
      Tactic::Position::Type position;
      int number;
      Vector2f positionOnField;
      Pose2f basePose; /**< The pose from the tactic position. */
      std::vector<Vector2f> baseArea; /**< The region in the Voronoi diagram of the base poses in the tactic. */
    };

  std::vector<float> getAssignmentCost(const Eigen::MatrixXf& costMatrix, const std::vector<std::size_t>& positionIndices);

  void assignPosition(Tactic::Position::Type positionToSet, Agent& agent);

  Tactic::Position firstReadyAssignPosition(std::vector<Tactic::Position> positions);

};
