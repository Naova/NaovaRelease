/**
 * This module provides a rating over the current field, and information about which field positions are currently better.
 * This is then later used to decide the dribble direction, kick direction in a duel or possible pass targets
 *
 * @file FieldScoreProvder.h
 * @author Robyn Girardeau
 *  */

#pragma once

#include "Representations/BehaviorControl/FieldScore.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Tools/Module/Module.h"
#include <vector>
#include "Representations/BehaviorControl/BallPlayerStrategy.h"
#include "Representations/BehaviorControl/FieldBall.h"

MODULE(FieldScoreProvider,
{,
  REQUIRES(FieldDimensions),
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(ObstacleModel),
  REQUIRES(RobotPose),
  REQUIRES(TeamData),
  REQUIRES(FieldBall),
  REQUIRES(BallPlayerStrategy),
  PROVIDES(FieldScore),
  DEFINES_PARAMETERS(
  {,
    (int)(120) stepX,
    (int)(120) stepY,
    (int) minX,
    (int) maxX,
    (int) minY,
    (int) maxY,
    (int) width,
    (int) height,
    (int) numberOfPointsX,
    (int) numberOfPointsY,
    (int) numberOfPoints,
    (int)(50) drawPointRadius,
    (int) searchRadius,
  }),
});


class FieldScoreProvider : public FieldScoreProviderBase
{
public:
  void update(FieldScore& FieldScore) override;
  FieldScoreProvider();

private:
  void drawPoint(int x, int y, double currentScore);
  int indexFromCoordinate(int x, int y);
  bool addScore(FieldScore &fieldScore, Vector2f &source, int x, int y);

};
