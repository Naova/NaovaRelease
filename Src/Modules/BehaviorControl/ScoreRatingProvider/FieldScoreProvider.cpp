/**
 * @file FieldScoreProvider.cpp
 * @author Robyn Girardeau
 */

#include "FieldScoreProvider.h"
#include "Tools/BehaviorControl/BehaviorUtilities.h"
#include "Tools/Debugging/DebugDrawings.h"

MAKE_MODULE(FieldScoreProvider, behaviorControl);

FieldScoreProvider::FieldScoreProvider() {
    minX = static_cast<int>(theFieldDimensions.xPosOwnGroundLine);
    maxX = static_cast<int>(theFieldDimensions.xPosOpponentGroundLine);
    minY = static_cast<int>(theFieldDimensions.yPosRightSideline);
    maxY = static_cast<int>(theFieldDimensions.yPosLeftSideline);
    width = maxX - minX;
    height = maxY - minY;
    numberOfPointsX = (width / stepX);
    numberOfPointsY = (height / stepY);
    numberOfPoints = numberOfPointsX * numberOfPointsY;
    searchRadius = 7 * stepX;
}

void FieldScoreProvider::update(FieldScore &fieldScore) {
    DECLARE_DEBUG_DRAWING("representation:FieldScoreProvider:scoreOnField", "drawingOnField");
    DEBUG_RESPONSE("representation:FieldScoreProvider:fieldScore");

    if (fieldScore.scores.size() != numberOfPoints) {
        fieldScore.scores.resize(numberOfPoints);
    }

    Vector2f source = Vector2f(theFieldBall.positionOnField);

    double cos_rotation = std::cos(theRobotPose.rotation);
    double sin_rotation = std::sin(theRobotPose.rotation);

    // Teammates
    for (int x = minX; x < maxX; x += stepX) {
        for (int y = minY; y < maxY; y += stepY) {
            for (Obstacle obstacle : theObstacleModel.obstacles) {
                if (obstacle.type == Obstacle::goalpost)
                    continue;

                Vector2f obstaclePos = Vector2f(
                    (obstacle.center.x() * cos_rotation - obstacle.center.y() * sin_rotation) + theRobotPose.translation.x(),
                    (obstacle.center.x() * sin_rotation + obstacle.center.y() * cos_rotation) + theRobotPose.translation.y());
                if (abs(obstaclePos.x() - x) > searchRadius)
                    continue;
                if (abs(obstaclePos.y() - y) > searchRadius)
                    continue;

                if (addScore(fieldScore, source, x, y))
                    break;
            }
        }
    }

    // Goal
    for (int x = theFieldDimensions.xPosOpponentGoalArea; x < theFieldDimensions.xPosOpponentGoal; x += stepX) {
        for (int y = theFieldDimensions.yPosRightGoalArea; y < theFieldDimensions.yPosLeftGoalArea; y += stepY) {
            addScore(fieldScore, source, x, y);
        }
    }
}

int FieldScoreProvider::indexFromCoordinate(int x, int y) {
    x = (x - minX) / stepX;
    y = (y - minY) / stepY;
    return x + y * numberOfPointsX;
}

bool FieldScoreProvider::addScore(FieldScore &fieldScore, Vector2f &source, int x, int y) {
    Vector2f target = Vector2f(x, y);
    double score = BehaviorUtilities::score(source, target, theObstacleModel.obstacles, theRobotPose);

    int index = indexFromCoordinate(x, y);
    fieldScore.scores[index] = score;

    drawPoint(x, y, score);
    return true;
}

void FieldScoreProvider::drawPoint(int x, int y, double score) {
    ColorRGBA color(0, 0, 0, 0);
    color.r = static_cast<unsigned char>((score) * 255.);
    color.g = static_cast<unsigned char>((1 - score) * 255.);
    color.a = static_cast<unsigned char>(score * 255.);
    LARGE_DOT("representation:FieldScoreProvider:scoreOnField",
              x, y, ColorRGBA(), color);
}
