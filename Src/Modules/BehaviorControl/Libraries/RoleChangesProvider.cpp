/**
 * @file RoleChanges.cpp
 */

#include "RoleChangesProvider.h"
#include "Tools/Math/Geometry.h"

MAKE_MODULE(RoleChangesProvider, behaviorControl);

void RoleChangesProvider::update(RoleChanges& roleChanges)
{
    if(isActive)
    {
        if (theFrameInfo.getTimeSince(roleChanges.lastRoleChange) > roleChangeDelay || roleChanges.lastRoleChange == 0)
        {
            changeRole(roleChanges, theBehaviorStatus.role);
            roleChanges.lastRoleChange = theFrameInfo.time;
        }
    }
    else
    {
        roleChanges.nextRole = theBehaviorStatus.role;
    }
}


void RoleChangesProvider::changeRole(RoleChanges& roleChanges, Role::RoleType roleType)
{
    switch (roleType)
    {
        case Role::striker:
            changeStriker(roleChanges);
            break;
        case Role::supporter:
            changeSupporter(roleChanges);
            break;
        case Role::rightDefender:
        case Role::leftDefender:
            changeDefender(roleChanges, roleType);
            break;
    }
}

void RoleChangesProvider::changeStriker(RoleChanges& roleChanges)
{
    if (theLibCodeRelease.nbOfStriker >= 1 && !theLibCodeRelease.closerToTheBall)
    {
        roleChanges.nextRole = getMissingRole();   
    }
    else
        roleChanges.nextRole = Role::striker;
}

void RoleChangesProvider::changeSupporter(RoleChanges& roleChanges)
{
    if ((needToChangeStriker() || theLibCodeRelease.nbOfStriker == 0))
    {
        roleChanges.nextRole = Role::striker;
    }
    else if ((theLibCodeRelease.nbOfRightDefender == 0 || isCloserToOwnGoal(Role::rightDefender)))
    {
        roleChanges.nextRole = Role::rightDefender;
    }
    else if ((theLibCodeRelease.nbOfLeftDefender == 0 || isCloserToOwnGoal(Role::leftDefender)))
    {
        roleChanges.nextRole = Role::leftDefender;
    }
    else
        roleChanges.nextRole = Role::supporter;
}

void RoleChangesProvider::changeDefender(RoleChanges& roleChanges, Role::RoleType roleType)
{
    if (theLibCodeRelease.nbOfRightDefender >= 1 && roleType == Role::rightDefender)
    {
        if (!isCloserToOwnGoal(roleType))
        {
            roleChanges.nextRole = getMissingRole();
        }
    }
    else if (theLibCodeRelease.nbOfLeftDefender >= 1 && roleType == Role::leftDefender)
    {
        if (!isCloserToOwnGoal(roleType))
        {
            roleChanges.nextRole = getMissingRole();
        }
    }
    // regarde si le défenseur droit est plus à gauche que le défenseur gauche
    // else if (roleType == Role::rightDefender && isFurtherSideDefender(Role::leftDefender))
    // {
    //     roleChanges.nextRole = Role::leftDefender;
    // }
    // regarde si le défenseur gauche est plus à droite que le défenseur droit
    // else if (roleType == Role::leftDefender && isFurtherSideDefender(Role::rightDefender))
    // {
    //     roleChanges.nextRole = Role::rightDefender;
    // }
    else if (needToChangeStriker())
    {
        roleChanges.nextRole = Role::striker;
    }
    else if (theLibCodeRelease.nbOfDefender == 1)
    {
        if (roleType == Role::leftDefender && theLibCodeRelease.nbOfStriker == 0 && theLibCodeRelease.nbOfSupporter == 0)
        {
            roleChanges.nextRole = Role::striker;
        }
        else
            roleChanges.nextRole = roleType;
    }
    else if (theLibCodeRelease.nbOfDefender == 0)
    {
        if (theLibCodeRelease.nbOfStriker == 0 && theLibCodeRelease.nbOfSupporter == 0)
        {
            roleChanges.nextRole = Role::striker;
        }
        else
            roleChanges.nextRole = roleType;
    }

    // TODO regarder pour changer comment ça fonctionne lorsqu'on aura implémenter le role defender lorsqu'il reste seulement un defender sur le jeu.
}

bool RoleChangesProvider::needToChangeStriker()
{
    bool isBallInEnemyZone = Geometry::isPointInsideRectangle2(Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosLeftSideline), 
                                                                Vector2f(theFieldDimensions.xPosOpponentGroundline, theFieldDimensions.yPosRightSideline), 
                                                                theTeamBallModel.position);
                                                                
    if (isBallInEnemyZone && theLibCodeRelease.closerToTheBall)
        return true;
    
    return false;
}

Role::RoleType RoleChangesProvider::getMissingRole()
{
    if (theLibCodeRelease.nbOfStriker == 0)
        return Role::striker;

    if (theLibCodeRelease.nbOfRightDefender == 0)
        return Role::rightDefender;

    if (theLibCodeRelease.nbOfLeftDefender == 0)
        return Role::leftDefender;
        
    if (theLibCodeRelease.nbOfSupporter == 0)
        return Role::supporter;

    return theBehaviorStatus.role;
}

bool RoleChangesProvider::isCloserToOwnGoal(Role::RoleType roleType)
{
    double teammateDistanceToOwnGoal = 0.0;

    distanceToOwnGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosCenterGoal)).norm();

    for(auto const& teammate : theTeamData.teammates)
    {
        if (teammate.theBehaviorStatus.role == roleType)
        {
            teammateDistanceToOwnGoal = (teammate.theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, theFieldDimensions.yPosCenterGoal)).norm();

            if(distanceToOwnGoal > teammateDistanceToOwnGoal)
                return false;    
            else if(distanceToOwnGoal == teammateDistanceToOwnGoal)
            {
                // robot number *20 pour avoir une plus grande différence entre 2 robots avec la même distance.
                if (distanceToOwnGoal - theRobotInfo.number*20 > teammateDistanceToOwnGoal - teammate.number*20)
                    return false;
            }  
        }
    }
    return true;
}

// Fonction si le défenseur est plus loin que l'autre défenseur.
// bool RoleChangesProvider::isFurtherSideDefender(Role::RoleType roleType)
// {
//     for(auto const& teammate : theTeamData.teammates)
//     {
//         if (teammate.theBehaviorStatus.role == roleType)
//         {
//             if (teammate.theBehaviorStatus.role == Role::leftDefender)
//             {
//                 if(theRobotPose.translation.y() > teammate.theRobotPose.translation.y())
//                     return true;
//                 else if(theRobotPose.translation.y() == teammate.theRobotPose.translation.y())
//                 {
//                     // robot number *20 pour avoir une plus grande différence entre 2 robots avec la même distance.
//                     if (theRobotPose.translation.y() - theRobotInfo.number*200 > teammate.theRobotPose.translation.y() - teammate.number*200)
//                         return true;
//                 }
//             }
//             else if (teammate.theBehaviorStatus.role == Role::rightDefender)
//             {
//                 if(theRobotPose.translation.y() < teammate.theRobotPose.translation.y())
//                     return true;
//                 else if(theRobotPose.translation.y() == teammate.theRobotPose.translation.y())
//                 {
//                     // robot number *20 pour avoir une plus grande différence entre 2 robots avec la même distance.
//                     if (theRobotPose.translation.y() - theRobotInfo.number*200 < teammate.theRobotPose.translation.y() - teammate.number*200)
//                         return true;
//                 }
//             }
//         }
//     }
//     return false;
// }