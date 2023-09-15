/**
 * @file Representations/BehaviorControl/BehaviorStatus.h
 * The file declares a struct that contains data about the current behavior state.
 * @author Andreas Stolpmann
 */

#pragma once

#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/TimeToReachBall.h"
#include "Representations/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"
#include "Tools/Debugging/DebugDrawings3D.h"
#include "Tools/Settings.h"

/**
 * @struct BehaviorStatus
 * A struct that contains data about the current behavior state.
 */
STREAMABLE(BehaviorStatus, COMMA public NaovaMessageParticule<idBehaviorStatus>
{
  ENUM(Activity,
  {,
    unknown,
    blocking,
    duel,
    dribble,
    dribbleDuel,
    searchForBall,
    searchForBallAtRecentPosition,
    goToBall,
    takingPosition,
    kick,
    guardGoal,
    catchBall,
    standAndWait,
    passing,
    gettingUp,
    turn,
    walkNextToKeeper,
    kickoff,

    waving,
  });
  /** NaovaMessageParticle functions */
  void operator >> (NaovaMessage& m) const override;
  void operator << (const NaovaMessage& m) override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  /** Draw representation. */
  void draw() const,

  ((Role) RoleType)(undefined) role,
  (Activity)(unknown) activity, /**< What is the robot doing in general? */
  (TimeToReachBall) timeToReachBall,
  (int)(-1) passTarget,
});

inline void BehaviorStatus::operator >> (NaovaMessage& m) const
{
  m.theNaovaStandardMessage.currentlyPerformingRole = role;
  m.theBHumanArbitraryMessage.queue.out.bin << activity;
  m.theBHumanArbitraryMessage.queue.out.finishMessage(id());
  timeToReachBall >> m;
  // m.theNaovaStandardMessage.passTarget = static_cast<int8_t>(passTarget);

  // m.theNaovaStandardMessage.keeperIsPlaying = role == Role::keeper &&
  //     (activity == BehaviorStatus::goToBall || activity == BehaviorStatus::kick || activity == BehaviorStatus::duel);
}

inline void BehaviorStatus::operator << (const NaovaMessage& m)
{
  role = m.theNaovaStandardMessage.currentlyPerformingRole;
  // activity = m.theNaovaStandardMessage.keeperIsPlaying && m.theNaovaSPLStandardMessage.playerNum == 1 ? BehaviorStatus::goToBall : unknown;
  timeToReachBall << m;
  // passTarget = static_cast<int>(m.theNaovaStandardMessage.passTarget);
}

inline bool BehaviorStatus::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());
  m.bin >> activity;
  return true;
}

inline void BehaviorStatus::draw() const
{
  DEBUG_DRAWING3D("representation:BehaviorStatus", "robot")
  {
    int pNumber = Global::getSettings().playerNumber;
    if(role == Role::undefined)
      pNumber = 0;
    else if(role == Role::keeper)
      pNumber = 1;
    else if(role == Role::rightDefender)
      pNumber = 2;
    else if(role == Role::leftDefender)
      pNumber = 3;
    else if(role == Role::supporter)
      pNumber = 4;
    else if(role == Role::striker)
      pNumber = 5;
    float centerDigit = (pNumber > 1) ? 50.f : 0;
    ROTATE3D("representation:BehaviorStatus", 0, 0, pi_2);
    DRAWDIGIT3D("representation:BehaviorStatus", pNumber, Vector3f(centerDigit, 0.f, 500.f), 80, 5, ColorRGBA::white);
  }
}