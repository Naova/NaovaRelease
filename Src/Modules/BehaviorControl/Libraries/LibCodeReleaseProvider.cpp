/**
 * @file LibCodeRelease.cpp
 */

#include "LibCodeReleaseProvider.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(LibCodeReleaseProvider, behaviorControl);

void LibCodeReleaseProvider::update(LibCodeRelease& libCodeRelease)
{
    if(theGameInfo.state != currentGameState)
        getLastGameState(libCodeRelease);

    libCodeRelease.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen);
    libCodeRelease.ballModelIsValid = theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) < 500;
    libCodeRelease.angleToOppGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    libCodeRelease.angleToOwnGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();

    libCodeRelease.closerToTheBall = isCloserToTheBall();
    libCodeRelease.closerToTheBallDef = isCloserToTheBallDef();
    
    if (theBehaviorStatus.role == Role::rightDefender)
        libCodeRelease.defenderCloserToTheBall = specificTeammateIsCloserToBall(Role::leftDefender);
    if (theBehaviorStatus.role == Role::leftDefender)
        libCodeRelease.defenderCloserToTheBall = specificTeammateIsCloserToBall(Role::rightDefender);

    libCodeRelease.ballIsInGoalArea = isBallInGoalArea();
    libCodeRelease.robotIsInPenaltyArea = isRobotInPenaltyArea();
    libCodeRelease.between = [&](float value, float min, float max) -> bool
    {
        return value >= min && value <= max;
    };
    libCodeRelease.clamp = [&](float value, float min, float max) -> float
    {
        if(min > max)
        {
            float tmp = max;
            max = min;
            min = tmp;
        }
        if(value <= min)
            return min;
        else if(value >= max)
            return max;
        else
            return value;
    };

    countRoles(libCodeRelease);
    if (theBehaviorStatus.role == Role::rightDefender || theBehaviorStatus.role == Role::leftDefender)
        getDefenderDesiredPos(libCodeRelease);
    getDesiredPos(libCodeRelease);
    getRole(libCodeRelease);
}

void LibCodeReleaseProvider::getDesiredPos(LibCodeRelease& libCodeRelease){

    Vector2f midPointBallGoal(0.f, 0.f);
    Vector2f strikerPos(0.f, 0.f);
    Vector2f supporterPos(0.f, 0.f);
    Vector2f desiredKickPos(0.f, 0.f);
    float xPos = 0.0;
    float yPos = 0.0;

    //float xBall = (theRobotPose.inversePose * theBallModel.estimate.position).x();
    //float yBall = (theRobotPose.inversePose * theBallModel.estimate.position).y();
    //float m = (yBall-0)/(xBall-theFieldDimensions.xPosOwnGroundline);

    const float margin = theFieldDimensions.fieldLinesWidth*4;
    // Ce Calcule la casse toute. Surement la division sur vecteur qui est trop lourde.. mais j'ai des bug de transition avec.
    //Vector2f midPointBallGoal = ((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f))
    //                     + theBallModel.estimate.position) / 2;

    Vector2f topLeftGoalAreaCorner(theFieldDimensions.xPosOwnGoalArea - margin, theFieldDimensions.yPosLeftGoalArea - margin);
    Vector2f topRightGoalAreaCorner(theFieldDimensions.xPosOwnGoalArea - margin, theFieldDimensions.yPosRightGoalArea + margin);
    Vector2f bottomLeftGoalAreaCorner(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoalArea - margin);
    Vector2f bottomRightGoalAreaCorner(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoalArea + margin);
    Vector2f centerGoal(theFieldDimensions.xPosOwnGroundline, 0.f);
    Vector2f intersection(0.f, 0.f);

    Geometry::Line ballLine;

    Geometry::Line topGoalArealine(theRobotPose.inversePose * topLeftGoalAreaCorner, theRobotPose.inversePose * topRightGoalAreaCorner);
    Geometry::Line leftGoalArealine(theRobotPose.inversePose * topLeftGoalAreaCorner, theRobotPose.inversePose * bottomLeftGoalAreaCorner);
    Geometry::Line rightGoalArealine(theRobotPose.inversePose * topRightGoalAreaCorner, theRobotPose.inversePose * bottomRightGoalAreaCorner);
    //float midGoalPoint = (theRobotPose.inversePose * Vector2f((theFieldDimensions.xPosOwnGroundline - theFieldDimensions.xPosOwnGoalArea)/2 + theFieldDimensions.xPosOwnGoalArea, 0.f)).x();

    float xZoneAvant = -(theFieldDimensions.centerCircleRadius+theFieldDimensions.fieldLinesWidth);
    //float xMiddle = theFieldDimensions.xPosOwnGoalArea/2;

    //float yLeftLimitField = theFieldDimensions.yPosLeftSideline;
    //float yZoneLeft = theFieldDimensions.yPosLeftGoal;
    //float yRightLimitField = theFieldDimensions.yPosRightSideline;
    //float yZoneRight = theFieldDimensions.yPosRightGoal;

//    Geometry::Line defenderAloneLine(Vector2f(theFieldDimensions.xPosOpponentPenaltyMark/2,), Vector2f() * topRightGoalAreaCorner);
//    Geometry::Line leftDefenderLine(theRobotPose.inversePose * topLeftPenaltyCorner, theRobotPose.inversePose * bottomLeftGoalAreaCorner);
//    Geometry::Line rightDefenderLine(theRobotPose.inversePose * topRightGoalAreaCorner, theRobotPose.inversePose * bottomRightGoalAreaCorner);
    Geometry::Line topDefenderline(theRobotPose.inversePose*Vector2f(xZoneAvant,theFieldDimensions.yPosLeftSideline),theRobotPose.inversePose*Vector2f(xZoneAvant,theFieldDimensions.yPosRightSideline));

    switch(theBehaviorStatus.role)
    {
        case Role::keeper:
            if(libCodeRelease.ballModelIsValid)
            {
                ballLine = Geometry::Line(theRobotPose.inversePose * theTeamBallModel.position, theRobotPose.inversePose * centerGoal);
            }
            else
            {
                ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose * centerGoal);
            }
            if(Geometry::checkIntersectionOfLines(ballLine.base, ballLine.direction, topGoalArealine.base, topGoalArealine.direction))
            {
                (void)Geometry::getIntersectionOfLines(ballLine, topGoalArealine, intersection);
            }
            else if(Geometry::checkIntersectionOfLines(ballLine.base, ballLine.direction, leftGoalArealine.base, leftGoalArealine.direction))
            {
                (void)Geometry::getIntersectionOfLines(ballLine, leftGoalArealine, intersection);
            }
            else if(Geometry::checkIntersectionOfLines(ballLine.base, ballLine.direction, rightGoalArealine.base, rightGoalArealine.direction)) {
                (void)Geometry::getIntersectionOfLines(ballLine, rightGoalArealine, intersection);
            }
//            else
//            {
//                intersection = Vector2f(midGoalPoint, 0.f);
//            }

            xPos = intersection.x();
            yPos = intersection.y();

            break;
        case Role::striker:
            supporterPos = findSupportPos();
            if(libCodeRelease.nbOfSupporter != 0 && supporterPos.x() > theRobotPose.translation.x() + theBallModel.estimate.position.x() && theRobotPose.translation.x() + theBallModel.estimate.position.x() < theFieldDimensions.xPosOpponentGroundline * 0.85)
            {
                libCodeRelease.desiredKickPos = Vector2f(supporterPos.x() + 500,supporterPos.y());
            }
            else
            {
                if (theRobotPose.translation.x() + theBallModel.estimate.position.x() >= theFieldDimensions.xPosOpponentPenaltyMark ){
                    if (theRobotPose.translation.x() + theBallModel.estimate.position.x() > theFieldDimensions.xPosOpponentPenaltyArea && theRobotPose.translation.y() + theBallModel.estimate.position.y() < theFieldDimensions.yPosLeftGoal-theFieldDimensions.goalPostRadius && theRobotPose.translation.y() + theBallModel.estimate.position.y() > theFieldDimensions.yPosRightGoal-theFieldDimensions.goalPostRadius){
                       libCodeRelease.desiredKickPos = Vector2f(theFieldDimensions.xPosOpponentGroundline, theRobotPose.translation.y() + theBallModel.estimate.position.y()); 
                    }
                    else {   
                        libCodeRelease.desiredKickPos = Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosCenterGoal);
                    }
                }
                else{
                    if (theRobotPose.translation.y() + theBallModel.estimate.position.y() >= 0){
                        libCodeRelease.desiredKickPos = Vector2f(theFieldDimensions.xPosOpponentGroundline, (theFieldDimensions.yPosLeftGoal + theFieldDimensions.yPosCenterGoal) /2);
                    }
                    else{
                        libCodeRelease.desiredKickPos = Vector2f(theFieldDimensions.xPosOpponentGroundline,(theFieldDimensions.yPosRightGoal + theFieldDimensions.yPosCenterGoal) /2);
                    }       
                }
                    
            }
            break; 
        case Role::supporter:
            strikerPos = findStrikerPos();

            // si on est en position defensive et assez loin du centre, on se place par rapport a la balle en x et y
            if (theTeamBallModel.position.x() < theFieldDimensions.xPosOwnGoal / 2.5f)
            {
                // si on est en y positif on se place en y negatif par rapport a la balle et vice versa
                if(theTeamBallModel.position.y() >= 0) {
                    xPos = (theRobotPose.inversePose * Vector2f(theTeamBallModel.position.x() + std::abs(theFieldDimensions.xPosOwnGoal / 5.f), theTeamBallModel.position.y() - 500.f)).x();
                    yPos = (theRobotPose.inversePose * Vector2f(theTeamBallModel.position.x() + std::abs(theFieldDimensions.xPosOwnGoal / 5.f), theTeamBallModel.position.y() - 500.f)).y();
                }
                else {
                    xPos = (theRobotPose.inversePose * Vector2f(theTeamBallModel.position.x() + std::abs(theFieldDimensions.xPosOwnGoal / 5.f), theTeamBallModel.position.y() + 500.f)).x();
                    yPos = (theRobotPose.inversePose * Vector2f(theTeamBallModel.position.x() + std::abs(theFieldDimensions.xPosOwnGoal / 5.f), theTeamBallModel.position.y() + 500.f)).y();
                }
            
                // on change l'angle du walkToTarget par rapport a la position sur le terrain, pas fonctionnel en ce moment
                if (Vector2f(xPos, yPos).norm() < 100.f)
                    libCodeRelease.supporterDesiredAngle = theBallModel.estimate.position.angle();
                else
                    libCodeRelease.supporterDesiredAngle = Vector2f(xPos, yPos).angle();

                // on kick vers la position du striker en standby
                libCodeRelease.supporterDesiredKickPos = Vector2f(theFieldDimensions.centerCircleRadius, 0.f);
            }
            // si on est en position defensive et mais trop proche du centre, on se place a une position fixe en x et par rapport a la balle en y
            else if (theTeamBallModel.position.x() < theFieldDimensions.xPosOwnGoal / 7.f)
            {
                // si on est en y positif on se place en y negatif par rapport a la balle et vice versa
                if(theTeamBallModel.position.y() >= 0) {
                    xPos = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGoal / 10.f, theTeamBallModel.position.y() - 500.f)).x();
                    yPos = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGoal / 10.f, theTeamBallModel.position.y() - 500.f)).y();
                }
                else {
                    xPos = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGoal / 10.f, theTeamBallModel.position.y() + 500.f)).x();
                    yPos = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGoal / 10.f, theTeamBallModel.position.y() + 500.f)).y();
                }
            
                // on change l'angle du walkToTarget par rapport a la position sur le terrain, pas fonctionnel en ce moment
                if (Vector2f(xPos, yPos).norm() < 100.f)
                    libCodeRelease.supporterDesiredAngle = theBallModel.estimate.position.angle();
                else
                    libCodeRelease.supporterDesiredAngle = Vector2f(xPos, yPos).angle();

                // on kick vers la position du striker en standby
                libCodeRelease.supporterDesiredKickPos = Vector2f(theFieldDimensions.centerCircleRadius, 0.f);
            }

            // si on est en position offensive et le striker est trop proche du but pour faire une passe on se place derriere lui
            else if (strikerPos.x() > theFieldDimensions.xPosOpponentGroundline * 0.6f)
            {
                // si le striker est a droite du supporter
                if(strikerPos.y() < theRobotPose.translation.y())
                {
                    // on se place a gauche du striker sauf si on depasse la limite du terrain
                    if ((strikerPos.y() + 1500.f) < theFieldDimensions.yPosLeftSideline)
                    {
                        xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() - 1000.f, strikerPos.y() + 1500.f)).x();
                        yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() - 1000.f, strikerPos.y() + 1500.f)).y();
                    }
                    else
                    {
                        xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() - 1000.f, strikerPos.y() - 1500.f)).x();
                        yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() - 1000.f, strikerPos.y() - 1500.f)).y();
                    }
                    
                }
                // si le striker est a gauche du supporter
                else
                {
                    // on se place a droite du striker sauf si on depasse la limite du terrain
                    if ((strikerPos.y() - 1500.f) > theFieldDimensions.yPosRightSideline)
                    {
                        xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() - 1000.f, strikerPos.y() - 1500.f)).x();
                        yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() - 1000.f, strikerPos.y() - 1500.f)).y();
                    }
                    else
                    {
                        xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() - 1000.f, strikerPos.y() + 1500.f)).x();
                        yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() - 1000.f, strikerPos.y() + 1500.f)).y();
                    }
                    
                }

                libCodeRelease.supporterDesiredKickPos = Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f);
            }
            // en position defensive, si le striker est a droite du supporter
            else if(strikerPos.y() < theRobotPose.translation.y())
            {
                // on se place a gauche du striker sauf si on depasse la limite du terrain
                if ((strikerPos.y() + 1500.f) < theFieldDimensions.yPosLeftSideline)
                {
                    xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 1000.f, strikerPos.y() + 1500.f)).x();
                    yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 1000.f, strikerPos.y() + 1500.f)).y();
                }
                else
                {
                    xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 1000.f, strikerPos.y() - 1500.f)).x();
                    yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 1000.f, strikerPos.y() - 1500.f)).y();
                }
                
                libCodeRelease.supporterDesiredKickPos = Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f);
            }
            // en position defensive, si le striker est a gauche du supporter
            else
            {
                // on se place a droite du striker sauf si on depasse la limite du terrain
                if ((strikerPos.y() - 1500.f) > theFieldDimensions.yPosRightSideline)
                {
                    xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 1000.f, strikerPos.y() - 1500.f)).x();
                    yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 1000.f, strikerPos.y() - 1500.f)).y();
                }
                else
                {
                    xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 1000.f, strikerPos.y() + 1500.f)).x();
                    yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 1000.f, strikerPos.y() + 1500.f)).y();
                }
                
                libCodeRelease.supporterDesiredKickPos = Vector2f(theFieldDimensions.xPosOpponentGoal, 0.f);
            }
            break;
    }
    desiredPos = Vector2f(xPos, yPos);
//    Geometry::clipPointInsideRectangle(theRobotPose.inversePose * (Vector2i)Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline),
//                                       theRobotPose.inversePose * (Vector2i)Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline),
//                                       desiredPos);
    libCodeRelease.desiredPos = desiredPos;
}

void LibCodeReleaseProvider::getDefenderDesiredPos(LibCodeRelease& libCodeRelease){

    float xPos = 0.0;
    float yPos = 0.0;

    Vector2f intersection(0.f, 0.f);

    Geometry::Line ballLine;

    float xZoneAvant = -(theFieldDimensions.centerCircleRadius+theFieldDimensions.fieldLinesWidth);
    
    float xZoneArriereWithBallDist = theFieldDimensions.xPosOwnGoalArea + (theFieldDimensions.fieldLinesWidth*2) + std::abs(((theRobotPose.inversePose * theTeamBallModel.position).x()/3));
    float xZoneArriereWithoutBallDist = theFieldDimensions.xPosOwnGoalArea + (theFieldDimensions.fieldLinesWidth*2);

    Geometry::Line topDefenderline(theRobotPose.inversePose*Vector2f(xZoneAvant,theFieldDimensions.yPosLeftSideline),theRobotPose.inversePose*Vector2f(xZoneAvant,theFieldDimensions.yPosRightSideline));
    Geometry::Line topFurtherDefenderline(theRobotPose.inversePose*Vector2f(xZoneAvant*2.f,theFieldDimensions.yPosLeftSideline),theRobotPose.inversePose*Vector2f(xZoneAvant*2.f,theFieldDimensions.yPosRightSideline));

    xPos = 0 ;
    yPos = 0;
    //0 just me, 1, i'm LEFT, 2 i'm RIGHT
    switch (theBehaviorStatus.role){
    // *** NE PAS EFFACER (Va être utile lorsqu'on aura le rôle pour un Defender) ***
        case Role::defender:
            if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOwnGoalArea,theFieldDimensions.yPosRightSideline))){
                intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoal))
                                    + (theRobotPose.inversePose*theTeamBallModel.position))/2 );

            }else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnGoalArea,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOwnGoalArea,theFieldDimensions.yPosLeftGoalArea))) {
                intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal))
                                    + (theRobotPose.inversePose*theTeamBallModel.position))/2 );
            }
            else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnGoalArea,theFieldDimensions.yPosRightGoalArea),Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightSideline))) {
                intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal))
                                    + (theRobotPose.inversePose*theTeamBallModel.position))/2 );

            }
            else if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosRightSideline)))
            {
                if(libCodeRelease.ballModelIsValid)
                {
                    ballLine = Geometry::Line(theRobotPose.inversePose*theTeamBallModel.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                }
                else
                {
                    ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                }
                (void)Geometry::getIntersectionOfLines(ballLine, topDefenderline, intersection);
            }    
            else{
                intersection = theRobotPose.inversePose*Vector2f(xZoneArriereWithBallDist,theFieldDimensions.yPosCenterGoal);
            }
            xPos = intersection.x();
            yPos = intersection.y();
    
            break;
        case Role::leftDefender:
            if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnGoalArea,theFieldDimensions.yPosLeftSideline),Vector2f(0.f,theFieldDimensions.yPosRightGoalArea)) && specificTeammateIsCloserToBall(Role::rightDefender)){
                intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoal))
                                    + (theRobotPose.inversePose*theTeamBallModel.position))/2 );
            }
            else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline), Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea)) && specificTeammateIsCloserToBall(Role::rightDefender))
            {
                intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal))
                                + (theRobotPose.inversePose*theTeamBallModel.position))/2 );
            }
            else if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.centerCircleRadius+theFieldDimensions.fieldLinesWidth,theFieldDimensions.yPosRightGoalArea)) && specificTeammateIsCloserToBall(Role::rightDefender)) 
            {
                if(libCodeRelease.ballModelIsValid)
                {
                    ballLine = Geometry::Line(theRobotPose.inversePose*theTeamBallModel.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                }
                else
                {
                    ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                }
                (void)Geometry::getIntersectionOfLines(ballLine, topFurtherDefenderline, intersection);
            }
            else if(isBallInZone(Vector2f(theFieldDimensions.centerCircleRadius+theFieldDimensions.fieldLinesWidth,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosRightGoalArea)) && specificTeammateIsCloserToBall(Role::rightDefender))
            {
                if(libCodeRelease.ballModelIsValid)
                {
                    ballLine = Geometry::Line(theRobotPose.inversePose*theTeamBallModel.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                }
                else
                {
                    ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                }
                (void)Geometry::getIntersectionOfLines(ballLine, topDefenderline, intersection);
            }
            else{
                if (isBallInZone(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoalArea), Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightSideline)))
                {
                    intersection = theRobotPose.inversePose*Vector2f(xZoneArriereWithoutBallDist, theFieldDimensions.yPosLeftGoal/3);
                }
                else
                {
                    intersection = theRobotPose.inversePose*Vector2f(xZoneArriereWithBallDist, theFieldDimensions.yPosLeftGoal/3);
                }
            }
            xPos = intersection.x();    
            yPos = intersection.y();
            break;
        case Role::rightDefender:
            if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnGoalArea,theFieldDimensions.yPosLeftGoalArea),Vector2f(0.f,theFieldDimensions.yPosRightSideline)) && specificTeammateIsCloserToBall(Role::leftDefender)){
                intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoal))
                                                            + (theRobotPose.inversePose*theTeamBallModel.position))/2 );

            }
            else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoalArea), Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightSideline)) && specificTeammateIsCloserToBall(Role::leftDefender)) {
                intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal))
                                + (theRobotPose.inversePose*theTeamBallModel.position))/2 );
            }
            else if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftGoalArea),Vector2f(theFieldDimensions.centerCircleRadius+theFieldDimensions.fieldLinesWidth,theFieldDimensions.yPosRightSideline)) && specificTeammateIsCloserToBall(Role::leftDefender)) 
            {
                if(libCodeRelease.ballModelIsValid)
                {
                    ballLine = Geometry::Line(theRobotPose.inversePose*theTeamBallModel.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightGoal/2.f));
                }
                else
                {
                    ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightGoal/2.f));
                }
                (void)Geometry::getIntersectionOfLines(ballLine, topFurtherDefenderline, intersection);
            }
            else if(isBallInZone(Vector2f(theFieldDimensions.centerCircleRadius+theFieldDimensions.fieldLinesWidth,theFieldDimensions.yPosLeftGoalArea),Vector2f(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosRightSideline)) && specificTeammateIsCloserToBall(Role::leftDefender)) {
                if(libCodeRelease.ballModelIsValid)
                {
                    ballLine = Geometry::Line(theRobotPose.inversePose*theTeamBallModel.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightGoal/2.f));
                }
                else
                {
                    ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightGoal/2.f));
                }
                (void)Geometry::getIntersectionOfLines(ballLine, topDefenderline, intersection);
            }
            else{
                if (isBallInZone(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline), Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea)))
                {
                    intersection = theRobotPose.inversePose*Vector2f(xZoneArriereWithoutBallDist, theFieldDimensions.yPosRightGoal/3);
                }
                else
                {
                    intersection = theRobotPose.inversePose*Vector2f(xZoneArriereWithBallDist, theFieldDimensions.yPosRightGoal/3);
                }
            }
            xPos = intersection.x();
            yPos = intersection.y();
            break;
    }
    defenderDesiredPos = Vector2f(xPos, yPos);
    libCodeRelease.defenderDesiredPos = defenderDesiredPos;
}

void LibCodeReleaseProvider::countRoles(LibCodeRelease& libCodeRelease) {
    // reset count
    libCodeRelease.nbOfDefender = 0;
    libCodeRelease.nbOfRightDefender = 0;
    libCodeRelease.nbOfLeftDefender = 0;
    libCodeRelease.nbOfKeeper = 0;
    libCodeRelease.nbOfStriker = 0;
    libCodeRelease.nbOfSupporter = 0;
    libCodeRelease.nbOfPlayers = 0;

    // Count teammate
    for(auto const& teammate : theTeamData.teammates)
    {
        switch (teammate.theBehaviorStatus.role)
        {
        case Role::rightDefender:
            libCodeRelease.nbOfDefender++;
            libCodeRelease.nbOfRightDefender++;
            libCodeRelease.nbOfPlayers++;
            break;
        case Role::leftDefender:
            libCodeRelease.nbOfDefender++;
            libCodeRelease.nbOfLeftDefender++;
            libCodeRelease.nbOfPlayers++;
            break;
        case Role::keeper:
            libCodeRelease.nbOfKeeper++;
            libCodeRelease.nbOfPlayers++;
            break;
        case Role::striker:
            libCodeRelease.nbOfStriker++;
            libCodeRelease.nbOfPlayers++;
            break;
        case Role::supporter:
            libCodeRelease.nbOfSupporter++;
            libCodeRelease.nbOfPlayers++;
            break;
        }
    }
}

//0 just me, 1, i'm LEFT, 2 i'm RIGHT
int LibCodeReleaseProvider::getMyNumberDefender()
{
    double teammatePositionY = 0.0;
    bool findOtherDefender = false;
    for(auto const& teammate : theTeamData.teammates)
    {
        if (teammate.theBehaviorStatus.role == Role::rightDefender || teammate.theBehaviorStatus.role == Role::leftDefender)
        {
//            double x = teammate.theRobotPose.translation.x();
            double y = teammate.theRobotPose.translation.y();
            findOtherDefender=true;
            teammatePositionY = y;
            if(theRobotPose.translation.y() < teammatePositionY)
            {
                return 2;
            }
        }
    }
    return findOtherDefender ? 1 : 0;
}

Vector2f LibCodeReleaseProvider::findStrikerPos()
{
    for(auto const& teammate : theTeamData.teammates)
    {
        if (teammate.theBehaviorStatus.role == Role::striker)
        {
            return teammate.theRobotPose.translation;
        }
    }
    return Vector2f(0.f, 0.f);
}

Vector2f LibCodeReleaseProvider::findSupportPos()
{
    for(auto const& teammate : theTeamData.teammates)
    {
        if (teammate.theBehaviorStatus.role == Role::supporter)
        {
            return teammate.theRobotPose.translation;
        }
    }
    return Vector2f(0.f, 0.f);
}

bool LibCodeReleaseProvider::isCloserToTheBall()
{
    if (!theTeamData.canSendMessages && (theRobotPose.inversePose*theTeamBallModel.position).norm() < 1500)
    {
        return true;
    }
    
    if (theTeamData.canSendMessages && theTeamBallModel.isValid)
    {
        double teammateDistanceToBall = 0.0;

        distanceToBall = (theRobotPose.inversePose*theTeamBallModel.position).norm();

        for(auto const& teammate : theTeamData.teammates)
        {
            if (teammate.theBehaviorStatus.role != Role::undefined)
            {
                teammateDistanceToBall = (teammate.theRobotPose.inversePose*theTeamBallModel.position).norm();

                if(distanceToBall > teammateDistanceToBall)
                    return false;
                else if(distanceToBall == teammateDistanceToBall)
                {
                    // robot number *20 pour avoir une plus grande différence entre 2 robots avec la même distance. 
                    if (distanceToBall - theRobotInfo.number*20 > teammateDistanceToBall - teammate.number*20)
                        return false;
                }  
            }  
        }
        return true;
    }
    
    return false;
}

bool LibCodeReleaseProvider::isCloserToTheBallDef() // fonction pour trouver le robot ideal pour bloquer un free kick
{
    double teammateDistanceToBall = 0.0f;

    distanceToBall = (theRobotPose.inversePose*theTeamBallModel.position).norm();
    
    // Robot n'est pas keeper
    if (theBehaviorStatus.role != Role::keeper)
    {
        // Le ballon est dans la moitié de notre zone la plus proche de notre but et le robot n'est pas trop proche du ballon
        if (theTeamBallModel.position.x() <= theFieldDimensions.xPosOwnGroundline * 0.5f &&
            (theRobotPose.inversePose*theTeamBallModel.position).norm() > theBehaviorParameters.safeDistance)
        {
            for(auto const& teammate : theTeamData.teammates)
            {
                // Le teammate n'est pas keeper et n'est pas trop proche du ballon
                if (teammate.theBehaviorStatus.role != Role::keeper &&
                    (teammate.theRobotPose.inversePose*theTeamBallModel.position).norm() > theBehaviorParameters.safeDistance)
                {
                    teammateDistanceToBall = (teammate.theRobotPose.inversePose*theTeamBallModel.position).norm();

                    if(distanceToBall > teammateDistanceToBall)
                        return false;
                    else if(distanceToBall == teammateDistanceToBall)
                    {
                        // robot number *100 pour avoir une plus grande différence entre 2 robots avec la même distance. 
                        if (distanceToBall - theRobotInfo.number*100 > teammateDistanceToBall - teammate.number*100)
                            return false;
                    }  
                }
            }
            return true;
        }
        else
        {
            // On ignore les robots trop proche de la balle
            if ((theRobotPose.translation.x() + theBehaviorParameters.safeDistance) <= theTeamBallModel.position.x())
            {
                for(auto const& teammate : theTeamData.teammates)
                {
                    // Teammate n'est pas keeper
                    if (teammate.theBehaviorStatus.role != Role::keeper)
                    {
                        // Teammate en haut du ballon
                        if ((teammate.theRobotPose.translation.x() + theBehaviorParameters.safeDistance) >= theTeamBallModel.position.x())
                            continue;
                        else
                            teammateDistanceToBall = (teammate.theRobotPose.inversePose*theTeamBallModel.position).norm();

                        if(distanceToBall > teammateDistanceToBall)
                            return false;
                        else if(distanceToBall == teammateDistanceToBall)
                        {
                            // robot number *100 pour avoir une plus grande différence entre 2 robots avec la même distance. 
                            if (distanceToBall - theRobotInfo.number*100 > teammateDistanceToBall - teammate.number*100)
                                return false;
                        }
                    }      
                }
                return true;
            }
            return false;
        }  
    }
    
    return false;
}

bool LibCodeReleaseProvider::specificTeammateIsCloserToBall(Role::RoleType teammateRole)
{
    double teammateDistanceToBall = 0.0;

    distanceToBall = theBallModel.estimate.position.norm();

    for(auto const& teammate : theTeamData.teammates)
    {
        if (teammate.theBehaviorStatus.role == teammateRole)
        {
            teammateDistanceToBall = teammate.theBallModel.estimate.position.norm();

            if(distanceToBall > teammateDistanceToBall)
                return false;    
            else if(distanceToBall == teammateDistanceToBall)
            {
                // robot number *20 pour avoir une plus grande différence entre 2 robots avec la même distance. 
                if (distanceToBall - theRobotInfo.number*20 > teammateDistanceToBall - teammate.number*20)
                    return false;
            }  
        }  
    }
    return true;
}

bool LibCodeReleaseProvider::isBallInGoalArea()
{
    return Geometry::isPointInsideRectangle2(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoalArea),
                                                Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea),
                                                theTeamBallModel.position);
}

bool LibCodeReleaseProvider::isRobotInPenaltyArea()
{
    return Geometry::isPointInsideRectangle2(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea),
                                                Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea),
                                                theRobotPose.translation);
}

bool LibCodeReleaseProvider::isBallInZone(Vector2f pointA,Vector2f pointB)
{
    return Geometry::isPointInsideRectangle2(pointA,pointB,theTeamBallModel.position);
}

void LibCodeReleaseProvider::getRole(LibCodeRelease& libCodeRelease)
{
    if (theRobotInfo.number == 1)
    {
        libCodeRelease.desiredRole = Role::keeper;
    }
    else if (libCodeRelease.nbOfStriker == 0)
    {
        libCodeRelease.desiredRole = Role::striker;
    }
    else if (libCodeRelease.nbOfRightDefender == 0)
    {
        libCodeRelease.desiredRole = Role::rightDefender;
    }
    else if (libCodeRelease.nbOfLeftDefender == 0)
    {
        libCodeRelease.desiredRole =  Role::leftDefender;
    }
    else if (libCodeRelease.nbOfSupporter == 0)
    {
        libCodeRelease.desiredRole =  Role::supporter;
    }
}

void LibCodeReleaseProvider::getLastGameState(LibCodeRelease& libCodeRelease)
{
    if(currentGameState != libCodeRelease.lastGameState)
    {
        libCodeRelease.lastGameState = currentGameState;
    }
    currentGameState = theGameInfo.state;
}

