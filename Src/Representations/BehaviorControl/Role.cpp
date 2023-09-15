/**
 * @file Representations/BehaviorControl/Role.h
 *
 * Implementation of the representation of a robot's behavior role
 *
 * @author Tim Laue, Andreas Stolpmann
 */

#include "Role.h"
#include "Representations/Infrastructure/CognitionStateChanges.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Settings.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"

bool Role::isGoalkeeper() const
{
  const bool goalKeeper = role == Role::keeper;
#ifndef NDEBUG
  if(goalKeeper
     && Blackboard::getInstance().exists("RobotInfo")
     && Blackboard::getInstance().exists("GameInfo")
     && Blackboard::getInstance().exists("CognitionStateChanges"))
  {
    auto robotNumber = static_cast<RobotInfo&>(Blackboard::getInstance()["RobotInfo"]).number;
    auto gamePhase = static_cast<GameInfo&>(Blackboard::getInstance()["GameInfo"]).gamePhase;
    auto lastgamePhase = static_cast<CognitionStateChanges&>(Blackboard::getInstance()["CognitionStateChanges"]).lastSecondaryGameState;
    if(gamePhase == lastgamePhase)
    {
      ASSERT(robotNumber == 1 || gamePhase == GAME_PHASE_PENALTYSHOOT);
    }
  }
#endif
  return goalKeeper;
}

void Role::draw() const
{
  DEBUG_DRAWING("representation:Role", "drawingOnField")
  {
    DRAWTEXT("representation:Role", -50, 250, 150, ColorRGBA::white, getName(role));
  }

  DEBUG_DRAWING3D("representation:Role3D", "robot")
  {
    static const ColorRGBA colors[numOfRoleTypes] =
    {
      ColorRGBA::black,
      ColorRGBA::blue,
      ColorRGBA::red,
      ColorRGBA::white,
      ColorRGBA::green,
      ColorRGBA::black
    };

    int pNumber = Global::getSettings().playerNumber;
    int r(role);
    r = r > 8 ? 8 : r;
    float centerDigit = (pNumber > 1) ? 180.f : 120.0f;
    ROTATE3D("representation:Role3D", 0, 0, pi_2);
    DRAWDIGIT3D("representation:Role3D", r, Vector3f(centerDigit, 0.0f, 500.f), 80, 5, colors[role]);
  }
}

void TeammateRoles::operator >> (NaovaMessage& m) const
{
  // for(size_t i = 0; i < sizeof(m.theNaovaStandardMessage.roleAssignments); ++i)
  //   m.theNaovaStandardMessage.roleAssignments[i] = (*this)[i + 1];
}

void TeammateRoles::operator<< (const NaovaMessage& m)
{
  // for(size_t i = 0; i < sizeof(m.theNaovaStandardMessage.roleAssignments); ++i)
  //   if(m.theNaovaStandardMessage.roleAssignments[i] != Role::undefined)
  //     (*this)[i + 1] = m.theNaovaStandardMessage.roleAssignments[i];
}

Role::RoleType& TeammateRoles::operator[](const size_t i)
{
  while(roles.size() <= i)
    roles.push_back(Role::RoleType::none);
  return roles[i];
}

Role::RoleType TeammateRoles::operator[](const size_t i) const
{
  if(roles.size() <= i)
    return Role::RoleType::none;
  return roles[i];
}

const char* TeammateRoles::getName(Role::RoleType e)
{
  return Role::getName(e);
}
