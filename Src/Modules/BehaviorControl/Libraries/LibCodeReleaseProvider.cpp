/**
 * @file LibCodeRelease.cpp
 */

#include "LibCodeReleaseProvider.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(LibCodeReleaseProvider, behaviorControl);

void LibCodeReleaseProvider::update(LibCodeRelease& libCodeRelease)
{
  libCodeRelease.timeSinceBallWasSeen = theFrameInfo.getTimeSince(theTeamBallModel.timeWhenLastSeen);
  libCodeRelease.timeSinceMissingTeammate = theFrameInfo.getTimeSince(libCodeRelease.timeWhenRepportMisssing);

  libCodeRelease.angleToOppGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  libCodeRelease.angleToOwnGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();

    libCodeRelease.setMissingTeammate = [&](bool isMissing) -> void
    {
        if(isMissing)
        {
            if (libCodeRelease.timeWhenRepportMisssing == 0)
            {
                libCodeRelease.timeWhenRepportMisssing = theFrameInfo.time;
            }
        }
        else{
            libCodeRelease.timeWhenRepportMisssing = 0;
        }
    };

    libCodeRelease.closerToTheBall = isCloserToTheBall();
    libCodeRelease.ballIsInPenaltyZone = isBallInPenaltyZone();
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
  getDesiredPos(libCodeRelease);

}

void LibCodeReleaseProvider::getDesiredPos(LibCodeRelease& libCodeRelease){

    Vector2f midPointBallGoal(0.f, 0.f);
    Vector2f strikerPos(0.f, 0.f);
    Vector2f supporterPos(0.f, 0.f);
    Vector2f desiredKickPos(0.f, 0.f);
    float xPos = 0.0;
    float yPos = 0.0;

    float xBall = (theRobotPose.inversePose * theBallModel.estimate.position).x();
    float yBall = (theRobotPose.inversePose * theBallModel.estimate.position).y();
    float m = (yBall-0)/(xBall-theFieldDimensions.xPosOwnGroundline);

    const float margin = theFieldDimensions.fieldLinesWidth*4;
    // Ce Calcule la casse toute. Surement la division sur vecteur qui est trop lourde.. mais j'ai des bug de transition avec.
    //Vector2f midPointBallGoal = ((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f))
    //                     + theBallModel.estimate.position) / 2;

    Vector2f topLeftPenalityCorner(theFieldDimensions.xPosOwnPenaltyArea - margin, theFieldDimensions.yPosLeftPenaltyArea - margin);
    Vector2f topRightPenalityCorner(theFieldDimensions.xPosOwnPenaltyArea - margin, theFieldDimensions.yPosRightPenaltyArea + margin);
    Vector2f bottomLeftPenalityCorner(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea - margin);
    Vector2f bottomRightPenalityCorner(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightPenaltyArea + margin);
    Vector2f centerGoal(theFieldDimensions.xPosOwnGroundline, 0.f);
    Vector2f intersection(0.f, 0.f);

    Geometry::Line ballLine;

    Geometry::Line topPenalityline(theRobotPose.inversePose * topLeftPenalityCorner, theRobotPose.inversePose * topRightPenalityCorner);
    Geometry::Line leftPenalityline(theRobotPose.inversePose * topLeftPenalityCorner, theRobotPose.inversePose * bottomLeftPenalityCorner);
    Geometry::Line rightPenalityline(theRobotPose.inversePose * topRightPenalityCorner, theRobotPose.inversePose * bottomRightPenalityCorner);
    //float midGoalPoint = (theRobotPose.inversePose * Vector2f((theFieldDimensions.xPosOwnGroundline - theFieldDimensions.xPosOwnPenaltyArea)/2 + theFieldDimensions.xPosOwnPenaltyArea, 0.f)).x();

    float xZoneAvant = -(theFieldDimensions.centerCircleRadius+theFieldDimensions.fieldLinesWidth);
    float xMiddle = theFieldDimensions.xPosOwnPenaltyArea/2;
    float xZoneArriere = theFieldDimensions.xPosOwnPenaltyArea + (theFieldDimensions.fieldLinesWidth*2);

    float yLeftLimitField = theFieldDimensions.yPosLeftSideline;
    float yZoneLeft = theFieldDimensions.yPosLeftGoal;
    float yZoneLeftCenter = libCodeRelease.safeDistance*4;
    float yRightLimitField = theFieldDimensions.yPosRightSideline;
    float yZoneRight = theFieldDimensions.yPosRightGoal;
    float yZoneRightCenter = -yZoneLeftCenter;

//    Geometry::Line defenderAloneLine(Vector2f(theFieldDimensions.xPosOpponentPenaltyMark/2,), Vector2f() * topRightPenalityCorner);
//    Geometry::Line leftDefenderLine(theRobotPose.inversePose * topLeftPenalityCorner, theRobotPose.inversePose * bottomLeftPenalityCorner);
//    Geometry::Line rightDefenderLine(theRobotPose.inversePose * topRightPenalityCorner, theRobotPose.inversePose * bottomRightPenalityCorner);
    Geometry::Line topDefenderline(theRobotPose.inversePose*Vector2f(xZoneAvant,theFieldDimensions.yPosLeftSideline),theRobotPose.inversePose*Vector2f(xZoneAvant,theFieldDimensions.yPosRightSideline));

    switch(theBehaviorStatus.role)
    {
        case Role::keeper:
            if(theTeamBallModel.isValid)
            {
                ballLine = Geometry::Line(theRobotPose.inversePose * theTeamBallModel.position, theRobotPose.inversePose * centerGoal);
            }
            else
            {
                ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose * centerGoal);
            }
            if(Geometry::checkIntersectionOfLines(ballLine.base, ballLine.direction, topPenalityline.base, topPenalityline.direction))
            {
                Geometry::getIntersectionOfLines(ballLine, topPenalityline, intersection);
            }
            else if(Geometry::checkIntersectionOfLines(ballLine.base, ballLine.direction, leftPenalityline.base, leftPenalityline.direction))
            {
                Geometry::getIntersectionOfLines(ballLine, leftPenalityline, intersection);
            }
            else if(Geometry::checkIntersectionOfLines(ballLine.base, ballLine.direction, rightPenalityline.base, rightPenalityline.direction)) {
                Geometry::getIntersectionOfLines(ballLine, rightPenalityline, intersection);
            }
//            else
//            {
//                intersection = Vector2f(midGoalPoint, 0.f);
//            }

            xPos = intersection.x();
            yPos = intersection.y();

            break;

        case Role::defender:
            xPos = 0 ;
            yPos = 0;
            switch (getMyNumberDefender()){
                case 0:
                    if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosRightSideline))){
                        intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftGoal))
                                         + (theRobotPose.inversePose*theTeamBallModel.position))/2 );

                    }else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosLeftPenaltyArea))) {
                        intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal))
                                         + (theRobotPose.inversePose*theTeamBallModel.position))/2 );
                    }
                    else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosRightPenaltyArea),Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightSideline))) {
                        intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal))
                                         + (theRobotPose.inversePose*theTeamBallModel.position))/2 );

                    }
                    else if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosRightSideline))) {
                        if(theTeamBallModel.isValid)
                        {
                            ballLine = Geometry::Line(theRobotPose.inversePose*theTeamBallModel.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                        }
                        else
                        {
                            ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                        }
                        Geometry::getIntersectionOfLines(ballLine, topDefenderline, intersection);
                    }else{
                        intersection = theRobotPose.inversePose*Vector2f(xZoneArriere,yZoneLeftCenter);
                    }
                    xPos = intersection.x();
                    yPos = intersection.y();
//
                    break;
                case 1:
                    if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosRightSideline))){
                        intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftGoal))
                                         + (theRobotPose.inversePose*theTeamBallModel.position))/2 );

                    }else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosLeftPenaltyArea))) {
                        intersection = theRobotPose.inversePose*Vector2f(xZoneArriere,theFieldDimensions.yPosRightGoal);
                    }
                    else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosRightPenaltyArea),Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightSideline))) {
                        intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftGoal))
                                         + (theRobotPose.inversePose*theTeamBallModel.position))/2 );

                    }
                    else if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosRightSideline))) {
                        if(theTeamBallModel.isValid)
                        {
                            ballLine = Geometry::Line(theRobotPose.inversePose*theTeamBallModel.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                        }
                        else
                        {
                            ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosLeftGoal/2.f));
                        }
                        Geometry::getIntersectionOfLines(ballLine, topDefenderline, intersection);
                    }else{
                        intersection = theRobotPose.inversePose*Vector2f(xZoneArriere,yZoneLeftCenter);
                    }
                    xPos = intersection.x();
                    yPos = intersection.y();
                    break;
                case 2:
                    if(isBallInZone(Vector2f(0.f,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosRightSideline))){
//                        midPointBallGoal = (theRobotPose.inversePose*((Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosLeftGoal))
//                                             + (theTeamBallModel.position))*0.75f );
//                        xPos = midPointBallGoal.x();
//                        yPos = midPointBallGoal.y();

//                        if(theTeamBallModel.isValid)
//                        {
//                            ballLine = Geometry::Line(theRobotPose.inversePose * theTeamBallModel.position, theRobotPose.inversePose * Vector2f(0,theFieldDimensions.yPosRightGoal/2.f));
//                        }
//                        else
//                        {
//                            ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose * Vector2f(0,theFieldDimensions.yPosRightGoal/2.f));
//                        }
//                        Geometry::getIntersectionOfLines(ballLine, topDefenderline, intersection);
//                        intersection = theRobotPose*intersection;
//                        midPointBallGoal = midPointBallGoal
//                        xPos = (theRobotPose.inversePose*Vector2f( midPointBallGoal.x()*0.75f,midPointBallGoal.y()*0.75f)).x();
//                        yPos = (theRobotPose.inversePose*Vector2f( midPointBallGoal.x()*0.75f,midPointBallGoal.y()*0.75f)).y();
                        intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightGoal))
                                                                  + (theRobotPose.inversePose*theTeamBallModel.position))/2 );

                    }
                    else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosLeftPenaltyArea))) {

                        intersection = theRobotPose.inversePose*Vector2f(xZoneArriere,theFieldDimensions.yPosLeftGoal);
                    }
                    else if(isBallInZone(Vector2f(theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosRightPenaltyArea),Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightSideline))) {
                        intersection = ((theRobotPose.inversePose*(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosRightGoal))
                                        + (theRobotPose.inversePose*theTeamBallModel.position))/2 );
                    }
                    else if(isBallInZone(Vector2f(0,theFieldDimensions.yPosLeftSideline),Vector2f(theFieldDimensions.xPosOpponentGoalPost,theFieldDimensions.yPosRightSideline))) {
                        if(theTeamBallModel.isValid)
                        {
                            ballLine = Geometry::Line(theRobotPose.inversePose*theTeamBallModel.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightGoal/2.f));
                        }
                        else
                        {
                            ballLine = Geometry::Line(theBallModel.estimate.position, theRobotPose.inversePose*Vector2f(theFieldDimensions.xPosOwnGroundline,theFieldDimensions.yPosRightGoal/2.f));
                        }
                        Geometry::getIntersectionOfLines(ballLine, topDefenderline, intersection);
                    }
                      else{
                        xPos = xZoneArriere;
                        yPos =yZoneLeftCenter;
                            intersection = theRobotPose.inversePose*Vector2f(xZoneArriere,yZoneRightCenter);
                    }
                    xPos = intersection.x();
                    yPos = intersection.y();
                    break;
            }
            break;
        case Role::striker:
            supporterPos = findSupportPos();
            if(supporterPos.x() > theRobotPose.translation.x() && theRobotPose.translation.x() < theFieldDimensions.xPosOpponentGroundline * 0.85)
            {
                libCodeRelease.angleToDesiredKick = (theRobotPose.inversePose * Vector2f(supporterPos.x() + 500,supporterPos.y())).angle();
            }
            else
            {
                libCodeRelease.angleToDesiredKick = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 200.f)).angle();
            }
            break;
        case Role::supporter:
            strikerPos = findStrikerPos();
            if(strikerPos.y() < theRobotPose.translation.y())
            {
                xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 700.f, strikerPos.y() + 900.f)).x();
                yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 700.f, strikerPos.y() + 900.f)).y();
            }
            else
            {
                xPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 700.f, strikerPos.y() - 900.f)).x();
                yPos = (theRobotPose.inversePose * Vector2f(strikerPos.x() + 700.f, strikerPos.y() - 900.f)).y();
            }
            break;
    }
    desiredPos = Vector2f(xPos, yPos);
//    Geometry::clipPointInsideRectangle(theRobotPose.inversePose * (Vector2i)Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftSideline),
//                                       theRobotPose.inversePose * (Vector2i)Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline),
//                                       desiredPos);
    libCodeRelease.desiredPos = desiredPos;
}

void LibCodeReleaseProvider::countRoles(LibCodeRelease& libCodeRelease) {
    // reset count
    libCodeRelease.nbOfDefender= 0;
    libCodeRelease.nbOfKeeper = 0;
    libCodeRelease.nbOfStriker = 0;
    libCodeRelease.nbOfSupporter = 0;

    // Count teammate
    for(auto const& teammate : theTeamData.teammates)
    {
        if (teammate.theBehaviorStatus.role == Role::defender)
            libCodeRelease.nbOfDefender= ++nbOfDefender;
        else if (teammate.theBehaviorStatus.role == Role::keeper)
            libCodeRelease.nbOfKeeper= ++nbOfKeeper;
        else if (teammate.theBehaviorStatus.role == Role::striker)
            libCodeRelease.nbOfStriker= ++nbOfStriker;
        else if (teammate.theBehaviorStatus.role == Role::supporter)
            libCodeRelease.nbOfSupporter= ++nbOfSupporter;
    }
}

//0 just me, 1, i'm LEFT, 2 i'm RIGHT
int LibCodeReleaseProvider::getMyNumberDefender()
{
    double teammatePositionY = 0.0;
    bool findOtherDefender = false;
    for(auto const& teammate : theTeamData.teammates)
    {
        if (teammate.theBehaviorStatus.role == Role::defender)
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
    double teammateDistanceToBall = 0.0;

    distanceToBall = theBallModel.estimate.position.norm();

    for(auto const& teammate : theTeamData.teammates)
    {
        if(theBehaviorStatus.role == Role::defender){
            if(teammate.theBehaviorStatus.role == Role::supporter ||
               teammate.theBehaviorStatus.role == Role::striker ||
                    teammate.theBehaviorStatus.role == Role::defender)
            {
                teammateDistanceToBall = teammate.theBallModel.estimate.position.norm();

                if(distanceToBall > teammateDistanceToBall)
                    return false;
            }
        }
        if(teammate.theBehaviorStatus.role == Role::supporter ||
                teammate.theBehaviorStatus.role == Role::striker)
        {
            teammateDistanceToBall = teammate.theBallModel.estimate.position.norm();

            if(distanceToBall > teammateDistanceToBall)
                return false;
        }
        
    }
    return true;
}

bool LibCodeReleaseProvider::isBallInPenaltyZone()
{
        return Geometry::isPointInsideRectangle2(Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosLeftPenaltyArea),
                                                 Vector2f(theFieldDimensions.xPosOwnPenaltyArea, theFieldDimensions.yPosRightPenaltyArea),
                                                 theTeamBallModel.position);
    }

bool LibCodeReleaseProvider::isBallInZone(Vector2f pointA,Vector2f pointB)
{
    return Geometry::isPointInsideRectangle2(pointA,pointB,theTeamBallModel.position);
}