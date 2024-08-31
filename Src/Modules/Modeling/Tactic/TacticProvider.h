/**
 * @file TacticProvider.h
 *
 * This file declares a module that <#...#>
 *
 * @author Olivier St-Pierre
 */

#pragma once

#include "Representations/BehaviorControl/CurrentTactic.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Tools/BehaviorControl/Strategy/Strategy.h"
#include "Tools/BehaviorControl/Strategy/Tactic.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Modeling/BallDropInModel.h"
#include "Tools/BehaviorControl/Strategy/SetPlay.h"
#include "Tools/BehaviorControl/Strategy/KickOff.h"
#include "Tools/BehaviorControl/Strategy/FreeKick.h"
#include "Tools/BehaviorControl/Strategy/PenaltyKick.h"
#include "Representations/Communication/RobotInfo.h"


#include "Tools/Module/Module.h"

MODULE(TacticProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(TeamData),
  REQUIRES(OwnTeamInfo),
  REQUIRES(OpponentTeamInfo),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldBall),
  REQUIRES(RobotDimensions),
  REQUIRES(GameInfo),
  REQUIRES(RobotInfo),
  REQUIRES(BallDropInModel),
  PROVIDES(CurrentTactic),
});

class TacticProvider : public TacticProviderBase
{
  public:
  TacticProvider();

  std::array<Strategy, Strategy::numOfTypes> strategies;
  std::array<Tactic, Tactic::numOfTypes> tactics;

  /**
   * This method is called when the representation provided needs to be updated.
   * @param theTactic The representation updated.
   */
  void update(CurrentTactic& theCurrentTactic) override;


  private:

  /**
    * Checks whether a set play can be started.
    * @param setPlay The set play to evaluate.
    * @return Whether the set play can be started.
    */
    [[nodiscard]] bool checkSetPlayStartConditions(SetPlay::Type setPlay) const;


  /**
 * Selects a set play.
 * @tparam SetPlayType The type of set play that must be chosen from.
 * @param setPlays The set of set plays of the given type.
 * @param random Whether selection is done randomly or pseudo-randomly based on GameController features.
 * @return A set play of the given type.
 */
  template<typename SetPlayType>
  SetPlay::Type selectNewSetPlay(const std::vector<Strategy::WeightedSetPlay<SetPlayType>>& setPlays, bool random) const;
  
  struct BallXTimestamps
  {
    unsigned lastTimeWhenNotAhead = 0; /** The last time when the ball was not ahead of a certain x threshold. */
    unsigned lastTimeWhenNotBehind = 0; /** The last time when the ball was not behind a certain x threshold. */
  };
  struct BallYTimestamps
  {
    unsigned lastTimeWhenNotAhead = 0; /** The last time when the ball was not ahead of a certain y threshold. */
    unsigned lastTimeWhenNotBehind = 0; /** The last time when the ball was not behind a certain y threshold. */
  };

  /** Updates \c ballXTimestamps. */
  void updateBallXTimestamps();
  void updateBallYTimestamps();

  std::unordered_map<float, BallXTimestamps> ballXTimestamps;
  std::unordered_map<float, BallYTimestamps> ballYTimestamps;
  std::vector<SetPlay*> setPlays;
  std::array<OwnKickOff, OwnKickOff::numOfTypes> ownKickOffs;
  std::array<OpponentKickOff, OpponentKickOff::numOfTypes> opponentKickOffs;
  std::array<OwnFreeKick, OwnFreeKick::numOfTypes> ownFreeKicks;
  std::array<OpponentFreeKick, OpponentFreeKick::numOfTypes> opponentFreeKicks;
  std::array<OwnPenaltyKick, OwnPenaltyKick::numOfTypes> ownPenaltyKicks;
  std::array<OpponentPenaltyKick, OpponentPenaltyKick::numOfTypes> opponentPenaltyKicks;

};
