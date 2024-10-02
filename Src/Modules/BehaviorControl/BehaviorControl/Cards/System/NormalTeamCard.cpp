/**
 * @file NormalTeamCard.cpp
 *
 * This file implements card that is active in situations where the normal team behavior is applied.
 *
 */

#include "Representations/BehaviorControl/TeamSkills.h"
#include "Tools/BehaviorControl/Framework/Card/TeamCard.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Configuration/FieldDimensions.h"

TEAM_CARD(NormalTeamCard,
{,
  CALLS(Role),
  CALLS(TeamActivity),
  CALLS(TeammateRoles),
  CALLS(TimeToReachBall),
  REQUIRES(TeamBehaviorStatus),
  REQUIRES(RobotInfo),
  REQUIRES(GameInfo),
  REQUIRES(WalkingEngineOutput),
  REQUIRES(FieldBall),
  REQUIRES(TeamData),
  REQUIRES(RobotPose),
  REQUIRES(FieldDimensions),
  DEFINES_PARAMETERS(
  {,
    (int)(5000) ballNotSeenTimeout,
    (int)(4) nbOfSupporters,
  }),
});
    
class NormalTeamCard : public NormalTeamCardBase
{
  bool preconditions() const override
  {
    return theGameInfo.competitionType != COMPETITION_TYPE_SHARED_AUTONOMY;
  }

  bool postconditions() const override
  {
    return theGameInfo.competitionType == COMPETITION_TYPE_SHARED_AUTONOMY;
  }

  void execute() override
  {
    theTeamActivitySkill(TeamBehaviorStatus::noTeam);

    if (theFieldBall.teamBallWasSeen(ballNotSeenTimeout))
    {
      theTimeToReachBallSkill(TimeToReachBall(getTimeToReachBall(theFieldBall.teamPositionRelative)));
    }
    else
    {
      theTimeToReachBallSkill(TimeToReachBall());
    }

    theTeammateRolesSkill(TeammateRoles());
    
    if (theGameInfo.state == STATE_INITIAL)
    {
      if(theRobotInfo.number == 5){
        theRoleSkill(PlayerRole(PlayerRole::RoleType::ballPlayer));
      }else if(theRobotInfo.number == 1){
        theRoleSkill(PlayerRole(PlayerRole::RoleType::goalkeeper));
      }else if(theRobotInfo.number == 2){
        theRoleSkill(PlayerRole(PlayerRole::RoleType::supporter2));
      }else if(theRobotInfo.number == 3){
        theRoleSkill(PlayerRole(PlayerRole::RoleType::supporter1));
      }else if(theRobotInfo.number == 4){
        theRoleSkill(PlayerRole(PlayerRole::RoleType::supporter0));
      }
    }
    else if (theGameInfo.state == STATE_PLAYING)
    {
      if (isBallPlayer())
      {
        theRoleSkill(PlayerRole(PlayerRole::RoleType::ballPlayer));
      }
      else if (theRobotInfo.number == 1)
      {
        theRoleSkill(PlayerRole(PlayerRole::RoleType::goalkeeper));
      }
      else
      {
        theRoleSkill(PlayerRole(static_cast<PlayerRole::RoleType>(PlayerRole::RoleType::supporter0 + getSupporterIndex())));
      }
    }
    else
    {
      theRoleSkill(PlayerRole(theTeamBehaviorStatus.role.role));
    }
  }

  bool isBallPlayer() const
  {
    for(auto const& teammate : theTeamData.teammates)
    {
      if(teammate.status != Teammate::PENALIZED)
      {
        unsigned int teammateTimeToReachBall = getTimeToReachBall(teammate.theRobotPose.inversePose*theFieldBall.teamPositionOnField);
        
        if(teammateTimeToReachBall < theTeamBehaviorStatus.timeToReachBall.timeWhenReachBall)
        {
          return false;
        }
        else if (teammateTimeToReachBall == theTeamBehaviorStatus.timeToReachBall.timeWhenReachBall)
        {
          if (!isRobotCloserToBall(teammate))
          {
            return false;
          }
        }
      }
    }
    return true;
  }

  int getSupporterIndex() const
  {
    int supporterIndex = 0;

    for (int i = -1; i < nbOfSupporters; i++)
    {
      for(auto const& teammate : theTeamData.teammates)
      {
        if(teammate.status != Teammate::PENALIZED && teammate.theTeamBehaviorStatus.role.supporterIndex() > i)
        {
          if(teammate.theRobotPose.translation.x() > theRobotPose.translation.x())
          {
            supporterIndex++;
            break;
          }
        }
      }
    }
    
    return supporterIndex;
  }

  bool isRobotCloserToBall(Teammate const& teammate) const
  {
    if (theRobotPose.translation.x() > theFieldBall.teamPositionOnField.x() 
      && teammate.theRobotPose.translation.x() < theFieldBall.teamPositionOnField.x())
    {
      // Robot is behind the ball and teammate is in front of the ball
      return false;
    }

    if (teammate.theRobotPose.translation.x() < theFieldBall.teamPositionOnField.x())
    {
      // Robot and teammate is in front of the ball
      if (theFieldBall.teamPositionOnField.y() < theFieldDimensions.yPosCenterGoal
        && theRobotPose.translation.y() > theFieldBall.teamPositionOnField.y()
        && teammate.theRobotPose.translation.y() < theFieldBall.teamPositionOnField.y())
      {
        // Ball is on the right side of the field and robot is on the left side of the ball when teammate is on the right side of the ball
        return false;
      }

      if (theFieldBall.teamPositionOnField.y() >= theFieldDimensions.yPosCenterGoal
        && theRobotPose.translation.y() < theFieldBall.teamPositionOnField.y()
        && teammate.theRobotPose.translation.y() > theFieldBall.teamPositionOnField.y())
      {
        // Ball is on the left side of the field and robot is on the right side of the ball when teammate is on the left side of the ball
        return false;
      }

      if ((teammate.theRobotPose.inversePose*theFieldBall.teamPositionOnField).norm()
        < (theFieldBall.teamPositionRelative).norm())
      {
        // Teammate is closer to the ball
        return false;
      }
    }

    return true;
  }

  unsigned int getTimeToReachBall(Vector2f vectorToReachBall) const
  {
    float timeToReachBall = std::max(1.f, std::max(std::abs(vectorToReachBall.x()) / theWalkingEngineOutput.maxSpeed.translation.x(),
          std::abs(vectorToReachBall.y()) / theWalkingEngineOutput.maxSpeed.translation.y()));
    
    return static_cast<unsigned int>(timeToReachBall);
  }
};

MAKE_TEAM_CARD(NormalTeamCard);
