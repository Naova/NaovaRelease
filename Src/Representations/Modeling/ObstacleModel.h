/**
 * @file ObstacleModel.h
 *
 * Declaration of struct ObstacleModel.
 *
 * @author Florian Maaß
 */
#pragma once

#include "Tools/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"
#include "Tools/Streams/Enum.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Modeling/Obstacle.h"

/**
 * @struct ObstacleModel
 *
 * A struct that represents all kind of obstacles seen by vision, detected by arm contact,
 * foot bumper contact.
 */

STREAMABLE(ObstacleModel, COMMA public NaovaMessageParticule<idObstacleModel>
{
  /** NaovaMessageParticule functions */
  void operator>>(NaovaMessage& m) const override;
  void operator<<(const NaovaMessage& m) override;

  ObstacleModel() = default;
  void draw() const;
  void verify() const,

  (std::vector<Obstacle>) obstacles, /**< List of obstacles (position relative to own pose) */
});
