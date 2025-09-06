
/**
 * @file DynamicSupporterPositionningProvider.cpp
 *
 * This file implements a module that dynamically changes a supporter's positionning
 * @author Jeremy Thu-Thon
 */

#include "DynamicSupporterPositionningProvider.h"


MAKE_MODULE(DynamicSupporterPositionningProvider, behaviorControl);

void DynamicSupporterPositionningProvider::update(DynamicSupporterPositionning& theDynamicSupporterPositionning)
{
    //prendre une première zone, celle fournie par la zone voronoi
    std::vector<Vector2f> area{theSupporterPositioning.baseArea};
    Vector2f bestPosition;

    //processus pour rétrécir la zone en fonction du point qui fournit le meilleur score. 
    for (size_t i = 0; i < steps.size(); i++)
    {
        findBounds(area);

        theDynamicSupporterPositionning.listePoints = scanArea(steps[i], area);

        //détermine la zone qui contient le meilleur score
        bestPosition = BehaviorUtilities::bestScorePosition(theDynamicSupporterPositionning.listePoints,
        Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosCenterGoal), theFieldBall.teamPositionOnField,
        theObstacleModel.obstacles, theRobotPose);

        std::vector<Vector2f> bestPositionSquare;

        //la nouvelle zone est un carré autour du point
        bestPositionSquare.push_back(bestPosition + Vector2f(steps[i]*stepMultiplier, steps[i]*stepMultiplier));
        bestPositionSquare.push_back(bestPosition + Vector2f(steps[i]*stepMultiplier, -steps[i]*stepMultiplier));
        bestPositionSquare.push_back(bestPosition + Vector2f(-steps[i]*stepMultiplier, steps[i]*stepMultiplier));
        bestPositionSquare.push_back(bestPosition + Vector2f(-steps[i]*stepMultiplier, -steps[i]*stepMultiplier));

        area = bestPositionSquare;
    }

    theDynamicSupporterPositionning.basePose = bestPosition;
    minX=std::numeric_limits<int>::max();
    minY=std::numeric_limits<int>::max();
    maxX=std::numeric_limits<int>::lowest();
    maxY=std::numeric_limits<int>::lowest();
}

// Fonction pour trouver les limites du polygone
void DynamicSupporterPositionningProvider::findBounds(std::vector<Vector2f> area) 
{
    for (const auto& point : area) {
        if (point.x() < minX) minX = (int)point.x();
        if (point.y() < minY) minY = (int)point.y();
        if (point.x() > maxX) maxX = (int)point.x();
        if (point.y() > maxY) maxY = (int)point.y();
    }
}

// Fonction pour scanner une zone et retourner les points à l'intérieur d'un polygone
std::vector<Vector2f> DynamicSupporterPositionningProvider::scanArea(float step, std::vector<Vector2f> area)
{
    std::vector<Vector2f> pointsInside;

    std::vector<Vector2f> goalArea{Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosRightGoalArea),
                                    Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosRightGoalArea),
                                    Vector2f(theFieldDimensions.xPosOpponentGoalArea, theFieldDimensions.yPosLeftGoalArea),
                                    Vector2f(theFieldDimensions.xPosOpponentGroundLine, theFieldDimensions.yPosLeftGoalArea)};

    // Boucle sur la zone donnée. Fait un scan sur la zone au complet
    for (float x = minX; x <= maxX; x+= step) {
        for (float y = minY; y <= maxY; y+= step)
         {
            Vector2f p = {x, y};

            float distanceBetweenBallAndPotentialPosition = (p-theFieldBall.positionOnField).norm();

            if (distanceBetweenBallAndPotentialPosition>1000 && distanceBetweenBallAndPotentialPosition < 3500 && Geometry::isPointInsidePolygon(p, area) && Geometry::isPointInsidePolygon(p, theSupporterPositioning.baseArea) && !Geometry::isPointInsidePolygon(p, goalArea))
            {
              pointsInside.push_back(p);
            }
        }
    }

    return pointsInside;
}