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
    (int)(2) timeToReachBallPenalty,
    (int)(1) notBallPlayerTimePenalty,
    (float)(3.5f) significantTimeDeltaThreashold,
  }),
});

class NormalTeamCard : public NormalTeamCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  void execute() override
  {
    theTeamActivitySkill(TeamBehaviorStatus::noTeam);

    if (theFieldBall.teamBallWasSeen(ballNotSeenTimeout))
    {
      theTimeToReachBallSkill(TimeToReachBall(static_cast<int>(getTimeToReachBall(theRobotPose.translation.x(), theFieldBall.teamPositionRelative))));
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
    float playerTimeToReachBall = getTimeToReachBall(theRobotPose.translation.x(), theRobotPose.inversePose*theFieldBall.teamPositionOnField);
    for(auto const& teammate : theTeamData.teammates)
    {
      float teammateTimeToReachBall = getTimeToReachBall(teammate.theRobotPose.translation.x(), teammate.theRobotPose.inversePose*theFieldBall.teamPositionOnField);
      float delta = teammateTimeToReachBall - playerTimeToReachBall;

      // If there's a significant difference between the player and the teammate
      if(abs(delta) > significantTimeDeltaThreashold)
      {
        if(teammateTimeToReachBall < playerTimeToReachBall)
        { 
          // A teammate is faster than the player, so this player is not ball player
          return false;
        }
      }
      else if (theRobotInfo.number < teammate.number)
      {
        // The robots are about the same time from the ball,
        // if another robot has a higher number this player is not ball player```
        return false;
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

  float getTimeToReachBall(float robotPoseX, Vector2f vectorToReachBall) const
  {
    int totalTimePenalty = 0;

    float timeToReachBall = std::max(std::abs(vectorToReachBall.x()) / theWalkingEngineOutput.maxSpeed.translation.x(),
          std::abs(vectorToReachBall.y()) / theWalkingEngineOutput.maxSpeed.translation.y());
    
    if (!(theTeamBehaviorStatus.role.playsTheBall()))
    {
      totalTimePenalty += notBallPlayerTimePenalty;
    }
    
    if (robotPoseX > theFieldBall.teamPositionOnField.x())  //Robot is ahead of the desired ball direction
    {
      totalTimePenalty += timeToReachBallPenalty;  //Time penalty for Robot on the opposite side of the ball
    }

    return timeToReachBall + static_cast<float>(totalTimePenalty);
  }
};

MAKE_TEAM_CARD(NormalTeamCard);
