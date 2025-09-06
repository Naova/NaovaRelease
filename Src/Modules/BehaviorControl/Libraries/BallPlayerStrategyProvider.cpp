/**
 * @file BallPlayerStrategyProvider.cpp
 *
 * This file implements a module that provides the BallPlayerStrategy representation
 *
 * @author Aissa, Elias, Marc-Olivier et  Catarina
 */

#include "BallPlayerStrategyProvider.h"
#include "Tools/BehaviorControl/BehaviorUtilities.h"
#include <limits>
#include <map>
#include "Tools/Math/Geometry.h"

MAKE_MODULE(BallPlayerStrategyProvider, behaviorControl);

void BallPlayerStrategyProvider::update(BallPlayerStrategy& theBallPlayerStrategy)
{
    draw(theBallPlayerStrategy);

    if(!theTeamBehaviorStatus.role.playsTheBall())
    {
        return;
    }
    KickOptionInfo kickOptionInfo;
    KickOptionInfo lastKickOptionInfo;
    Vector2f initialBallPosition = Vector2f(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
    float closestEnemyDistance = closestEnemyToDuel(theBallPlayerStrategy);
    

    if((initialBallPosition - theFieldBall.positionOnField).norm() > ballDifference)
    {
        lastKickOptionInfo.kickScore = std::numeric_limits<double>::lowest();
    }
    if (!isClearingTheBall(theBallPlayerStrategy))
    {
        if (closestEnemyDistance < distanceForDuel) 
        {
            theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::duel;
            lastStrategy = theBallPlayerStrategy.currentStrategy;
        }
        else
        {
            Vector2f robotAbsolutePosition = Vector2f(theRobotPose.translation.x(),theRobotPose.translation.y());

            kickOptionInfo = findBestKickTarget(kickAtGoalOptions);

            theBallPlayerStrategy.BPisBlockedInPenaltyArea = isBlockedNearOpponentGoal(kickOptionInfo);

            if(!(kickOptionInfo.kickScore > (lastKickOptionInfo.kickScore + scoreDifference)))
            {
                kickOptionInfo = lastKickOptionInfo;
            }
            else
            {
                lastKickOptionInfo = kickOptionInfo;                                                
            }

            double passScore = std::numeric_limits<double>::max();
            Vector2f teammateAbsolutePosition;

            if(theTeamData.teammates.size() != 0){
                teammateAbsolutePosition = findTeammateForPass(robotAbsolutePosition);
                passScore = passingScore(kickAtGoalOptions, teammateAbsolutePosition) * passWeight;
                
            }
            double kickScore = kickOptionInfo.kickScore * kickWeight;

            if(theGameInfo.getStateAsString() == "Corner Kick" || theGameInfo.getStateAsString() == "Kick In"){
               kickScore = std::numeric_limits<double>::max();
            }

            double dribbleScore = dribblingScore(theDribblePath.stepPositions, kickOptionInfo, kickAtGoalOptions) * dribbleWeight;

            if (teammateAbsolutePosition.x() < 0)
                passScore*=1000;

            nextAction(passScore, kickScore, dribbleScore, theBallPlayerStrategy, teammateAbsolutePosition, kickOptionInfo);
            
            
        }
    }
}


Vector2f BallPlayerStrategyProvider::findTeammateForPass(Vector2f robotAbsolutePosition)
{
    Vector2f localTeammateAbsolutePosition;
    Vector2f localTeammateSearchAbsolutePosition;
    
    double teammateDistance;
    Vector2f target = Vector2f(theFieldDimensions.xPosOwnFieldBorder, 0.f);
    Vector2f opponentGoalTarget = Vector2f(theFieldDimensions.xPosOpponentFieldBorder+200, 0.f);
    double positionScore = std::numeric_limits<double>::max();
    double computedScore;
    double teammateBestScore = std::numeric_limits<double>::max();
    double teammateActualScore;

    for(auto const& teammate : theTeamData.teammates)
    {
        localTeammateAbsolutePosition = Vector2f(teammate.theRobotPose.translation.x(), teammate.theRobotPose.translation.y()); 
        teammateDistance = (localTeammateAbsolutePosition-robotAbsolutePosition).norm();

        if (teammate.status != Teammate::PENALIZED && teammate.status != Teammate::FALLEN)
        {

            teammateActualScore = teammateScore(robotAbsolutePosition, localTeammateAbsolutePosition);
            if (teammateDistance<=passRange && teammateActualScore<teammateBestScore)
            {
                teammateBestScore = teammateActualScore;
                computedScore = BehaviorUtilities::score(localTeammateAbsolutePosition, opponentGoalTarget, theObstacleModel.obstacles, theRobotPose);
                
                if(computedScore < positionScore){
                    positionScore = computedScore;

                    target = localTeammateAbsolutePosition;
                }
               
            }
        }
    }
    
    if(target == Vector2f(theFieldDimensions.xPosOwnFieldBorder, 0.f))
    {
        if(theGameInfo.getStateAsString() == "Corner Kick" || theGameInfo.getStateAsString() == "Kick In"){
            return Vector2f(theFieldDimensions.xPosOpponentPenaltyMark, 0.f);
        }
        return Vector2f(theFieldDimensions.xPosOpponentFieldBorder, 0.f);
    }
    else
    {
        return target;
    }
}

double BallPlayerStrategyProvider::teammateScore(Vector2f robotPos, Vector2f teammatePos)
{
    double distToTeammate = (teammatePos - robotPos).norm(); // en mètres
    double distToGoal = (Vector2f(theFieldDimensions.xPosOpponentFieldBorder, 0.f) - teammatePos).norm(); // en mètres
    double score = distToTeammate + distToGoal;

    Vector2f pointA = robotPos;
    Vector2f pointB = teammatePos;
    Vector2f pointC = Vector2f(theFieldDimensions.xPosOpponentFieldBorder, 0.f);

    // Création de la ligne
    Geometry::Line line1;
    line1.base = pointA;
    line1.direction = pointB - pointA;

    Geometry::Line line2;
    line2.base = pointB;
    line2.direction = pointC - pointB;

    // Vérifie les obstacles proche de la ligne de passe
    for (const Obstacle& obstacle : theObstacleModel.obstacles)
    {
        if (obstacle.type != Obstacle::teammate)
        {
            Vector2f obsPos = theRobotPose * obstacle.center;
            double d1 = Geometry::getDistanceToLine(line1, obsPos);
            if (d1 < obstacleDistanceToLine) { // moins de 30 cm de la ligne de passe
                score += 10.0;
            }
            double d2 = Geometry::getDistanceToLine(line2, obsPos);
            if(d2 < obstacleDistanceToLine){
                score += 10.0;
            }
        }
    }

    return score; // Plus bas = meilleur
}
bool BallPlayerStrategyProvider::isClearingTheBall(BallPlayerStrategy& theBallPlayerStrategy)
{
    if (theFieldBall.positionOnField.x() >= -theFieldDimensions.centerCircleRadius)
        return false;

    theBallPlayerStrategy.currentStrategy = BallPlayerStrategy::Strategy::clear;

    const bool ballOnRightSide = theFieldBall.positionOnField.y() < 0;

    Vector2f initialClearPos = ballOnRightSide ? clearingPositionRight : clearingPositionLeft;
    double y_axis = initialClearPos.y(); // départ le long de l’axe horizontal
    const double stepDirection = ballOnRightSide ? 1.f : -1.f;

    Vector2f finalTarget = initialClearPos;

    double bestScore = std::numeric_limits<double>::max();

    //On itere 5 fois, car initialClearPos.y() --> +/- 2400mm et on fait des bonds de 480mm
    for (int i = 0; i < 5; ++i)
    {
        Vector2f testTarget(0.f, y_axis);

    
        double score = BehaviorUtilities::score(theFieldBall.positionOnField, testTarget,  theObstacleModel.obstacles, theRobotPose);

        if(score < bestScore){
            bestScore = score;
            finalTarget = testTarget;
        }
        y_axis += stepDirection * clearPositionStep;
    }

    theBallPlayerStrategy.targetForClear = finalTarget;
    return true;
}

BallPlayerStrategyProvider::KickOptionInfo BallPlayerStrategyProvider::findBestKickTarget(const std::vector<Vector2f>& kickAtGoalOptions)
{
    KickOptionInfo kickOptionInfo;        
    kickOptionInfo.kickScore = std::numeric_limits<double>::max();
    Vector2f robotAbsolutePosition = Vector2f(theRobotPose.translation.x(),theRobotPose.translation.y());
    bool isRobotInLeftBadShootingPosition = robotAbsolutePosition.x() > 0 && robotAbsolutePosition.y() > 1700 ? true : false;
    bool isRobotInRightBadShootingPosition = robotAbsolutePosition.x() > 0 && robotAbsolutePosition.y() < -1700 ? true : false;

    for(const Vector2f &target : kickAtGoalOptions)
    {

        if(isRobotInLeftBadShootingPosition && target == Vector2f(theFieldDimensions.xPosOpponentGroundLine, 500)){
            continue;
        }
        else if(isRobotInRightBadShootingPosition && target == Vector2f(theFieldDimensions.xPosOpponentGroundLine, -500)){
            continue;
        }
        

        double computedScore = BehaviorUtilities::score(theFieldBall.positionOnField, target, theObstacleModel.obstacles, theRobotPose);

        if (computedScore < kickOptionInfo.kickScore)
        {
            kickOptionInfo.kickScore = computedScore;
            kickOptionInfo.bestTarget = target;
        }
    }

    return kickOptionInfo;
}

double BallPlayerStrategyProvider::passingScore(const std::vector<Vector2f>& kickAtGoalOptions, Vector2f teammateAbsolutePosition)
{
    double passScore = std::numeric_limits<double>::max();
    
    if (teammateAbsolutePosition != Vector2f::Zero()) 
    {
        for(const Vector2f &target : kickAtGoalOptions)
        {
            double computedScore = BehaviorUtilities::score(teammateAbsolutePosition, target, theObstacleModel.obstacles, theRobotPose);
            double teammateComputedScore = BehaviorUtilities::score(theFieldBall.positionOnField, teammateAbsolutePosition, theObstacleModel.obstacles, theRobotPose);

            computedScore = (computedScore + teammateComputedScore)/2;

            if (computedScore < passScore)
            {
                passScore = computedScore;
            }
        }
    }

    return passScore;
}

double BallPlayerStrategyProvider::dribblingScore(std::vector<Vector2f> stepPositions, KickOptionInfo& kickOptionInfo, const std::vector<Vector2f>& kickAtGoalOptions)
{
    
    double dribblingScore = 0;
    double scoreCoefficient = firstDribbleIterationCoefficient;
    for(unsigned long i = 0; i < stepPositions.size(); i+=10){
        double kickScore = BehaviorUtilities::score(stepPositions[i], kickOptionInfo.bestTarget,theObstacleModel.obstacles, theRobotPose);

        Vector2f teammateAbsolutePosition = findTeammateForPass(stepPositions[i]);


        if(theTeamData.teammates.size()!=0 && teammateAbsolutePosition != Vector2f(std::numeric_limits<double>::max(), std::numeric_limits<double>::max())){
            double passScore = passingScore(kickAtGoalOptions, teammateAbsolutePosition);

                dribblingScore += (passScore+kickScore)/2 * scoreCoefficient;
                scoreCoefficient /= 2;

        }
        else{
        
                dribblingScore += (kickScore) * scoreCoefficient;
                scoreCoefficient /= 2;
        }
        
    }


    Vector2f opponentGoalCenter = Vector2f(theFieldDimensions.xPosOpponentGroundLine, 0.f);
    Vector2f ballCenter = theFieldBall.positionOnField;

    double distanceOfDribble = (opponentGoalCenter-ballCenter).norm();
    double dribbleDistancePenalty = x_axisDribbleDistanceBasis/distanceOfDribble;
    return dribblingScore * dribbleDistancePenalty;
}

void BallPlayerStrategyProvider::nextAction(double passScore, double kickScore, double dribbleScore, BallPlayerStrategy& theBallPlayerStrategy, Vector2f& teammate,KickOptionInfo& kickOptionInfo){
    std::map<BallPlayerStrategy::Strategy, double> scores = {
            {BallPlayerStrategy::Strategy::pass       , passScore },
            {BallPlayerStrategy::Strategy::kickAtGoal , kickScore },
            {BallPlayerStrategy::Strategy::dribble    , dribbleScore}
        };
    

    

    double bestScore = std::numeric_limits<double>::max();
    std::pair<BallPlayerStrategy::Strategy, double> bestPair;

    for(const auto& pair: scores){

        if(pair.second < bestScore ){
            bestScore = pair.second;
            bestPair = pair;
        }
    }


    // Si on passe de la strategie unknown, clear ou duel vers dribble, pass ou kick
    if(theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::unknown 
        || theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::clear 
        || theBallPlayerStrategy.currentStrategy == BallPlayerStrategy::Strategy::duel){
        theBallPlayerStrategy.currentStrategy = bestPair.first;
    }
    // Si on ne change pas de strategy, on compare le score avec un biais de cible (ex: passTargetBias) de maniere a ne pas changer la cible (cible de passe ou de kick) indefiniment
    else if(theBallPlayerStrategy.currentStrategy == bestPair.first){
        
        switch(bestPair.first){
                case BallPlayerStrategy::Strategy::pass:
                    if((theBallPlayerStrategy.targetForPass - theRobotPose.inversePose * teammate).norm() > passTargetBias){

                        theBallPlayerStrategy.targetForPass = teammate;

                    }
                    break;
                case BallPlayerStrategy::Strategy::kickAtGoal:
                    if((theBallPlayerStrategy.targetForKick - kickOptionInfo.bestTarget).norm() > goalTargetBias){

                        theBallPlayerStrategy.targetForKick = kickOptionInfo.bestTarget;
                    }
                    break;
                case BallPlayerStrategy::Strategy::dribble:
                    break;

            }
    }
    // Si on change de strategy (entre dribble, pass, kick), on compare le score avec un biais de strategie (strategyBias) de maniere a ne pas changer la strategy indefiniment
    else{
        
        if (bestPair.second < scores.find(theBallPlayerStrategy.currentStrategy)->second - strategyBias){
            theBallPlayerStrategy.currentStrategy = bestPair.first;
            switch(bestPair.first){
                case BallPlayerStrategy::Strategy::pass:
                    theBallPlayerStrategy.targetForPass = teammate;
                    break;
                case BallPlayerStrategy::Strategy::kickAtGoal:
                    theBallPlayerStrategy.targetForKick = kickOptionInfo.bestTarget;
                    break;
                case BallPlayerStrategy::Strategy::dribble:
                    break;

            }

        }
    }
}

float BallPlayerStrategyProvider::closestEnemyToDuel(BallPlayerStrategy& theBallPlayerStrategy)
{       
    float closestEnemyDistance = std::numeric_limits<float>::max();

    if(std::abs(theFieldBall.positionOnField.y()) < theFieldDimensions.yPosLeftSideline * 0.7f && std::abs(theFieldBall.positionOnField.x()) < theFieldDimensions.xPosOpponentGoalArea)
    {
        for(const Obstacle& obstacle : theObstacleModel.obstacles)
        {
            float obstacleDistance = obstacle.center.norm();
            Vector2f obstacleToball = (theRobotPose * obstacle.center) - theFieldBall.positionOnField;
            Vector2f kickDirectionToBall = (kickDirection - theFieldBall.positionOnField);
            float obstacleToKickAngle = kickDirectionToBall.angleTo(obstacleToball);

            if (obstacle.isOpponent() && obstacleDistance < closestEnemyDistance && abs(obstacleToKickAngle) < angleToConsiderDuel)
            {
                closestEnemyDistance = obstacleDistance;
                theBallPlayerStrategy.closestEnemy = obstacle;
            }
        }
    }
    return closestEnemyDistance;
}

void BallPlayerStrategyProvider::draw(BallPlayerStrategy& theBallPlayerStrategy) const
{
    DEBUG_DRAWING3D("module:BallPlayerStrategyProvider:strategy", "robot")
    {

        if (!theTeamBehaviorStatus.role.playsTheBall())
            return;
            
        ColorRGBA color = ColorRGBA::white;

        switch (theBallPlayerStrategy.currentStrategy)
        {
            case BallPlayerStrategy::Strategy::dribble:
                color = ColorRGBA::green;
                break;
            case BallPlayerStrategy::Strategy::kickAtGoal:
                color = ColorRGBA::red;
                break;
            case BallPlayerStrategy::Strategy::pass:
                color = ColorRGBA::magenta;
                break;
            case BallPlayerStrategy::Strategy::duel:
                color = ColorRGBA::yellow;
                break;
            case BallPlayerStrategy::Strategy::clear:
                color = ColorRGBA::cyan;
                break;
            default:
                color = ColorRGBA::white;
                break;
        }

        // The digit must be a modifiable value in order to be drawn
        int num = theRobotInfo.number;
        ROTATE3D("module:BallPlayerStrategyProvider:strategy", 0, 0, pi_2);
        DRAWDIGIT3D("module:BallPlayerStrategyProvider:strategy", num, Vector3f(50.f, 0.f, 500.f), 85, 8, color);
    }
}

bool BallPlayerStrategyProvider::isBlockedNearOpponentGoal(KickOptionInfo& kickOptionInfo)
{
    if((theFieldBall.positionOnField - kickOptionInfo.bestTarget).norm() <= maximumBlockingDistanceFromOpponentGoal){

        Vector2f obstaclePosition;
        double distBallToObstacle;
        for (Obstacle obstacle : theObstacleModel.obstacles)
        {

            obstaclePosition = theRobotPose * obstacle.center;
            distBallToObstacle = (theFieldBall.positionOnField - obstaclePosition).norm();
            
            if(obstaclePosition.x() >= theFieldBall.positionOnField.x() && distBallToObstacle < maximumBlockingDistanceFromOpponentPlayer){
                return true;
            }
        }
    }

    return false;
}
