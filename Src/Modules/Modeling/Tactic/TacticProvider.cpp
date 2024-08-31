/**
 * @file TacticProvider.cpp
 *
 * This file implements a module that <#...#>
 *
 * @author Olivier St-Pierre
 */

#include "TacticProvider.h"
#include "Representations/BehaviorControl/CurrentTactic.h"
#include "Tools/BehaviorControl/Strategy/Strategy.h"
#include "Tools/Libs/InExpr.h"
#include "Platform/File.h"

MAKE_MODULE(TacticProvider, behaviorControl);


TacticProvider::TacticProvider(){

    // Create the symbol map for loading behaviors from configuration files.
    std::unordered_map<std::string, float> symbols;
#define SET_SYMBOL(representation, member) symbols[#member] = (representation).member
    SET_SYMBOL(theFieldDimensions, xPosOwnFieldBorder);
    SET_SYMBOL(theFieldDimensions, xPosOwnGoal);
    SET_SYMBOL(theFieldDimensions, xPosOwnGoalPost);
    SET_SYMBOL(theFieldDimensions, xPosOwnGroundLine);
    SET_SYMBOL(theFieldDimensions, xPosOwnGoalArea);
    SET_SYMBOL(theFieldDimensions, xPosOwnPenaltyMark);
    SET_SYMBOL(theFieldDimensions, xPosOwnPenaltyArea);
    SET_SYMBOL(theFieldDimensions, xPosHalfWayLine);
    SET_SYMBOL(theFieldDimensions, xPosPenaltyStrikerStartPosition);
    SET_SYMBOL(theFieldDimensions, xPosOpponentPenaltyArea);
    SET_SYMBOL(theFieldDimensions, xPosOpponentPenaltyMark);
    SET_SYMBOL(theFieldDimensions, xPosOpponentGoalArea);
    SET_SYMBOL(theFieldDimensions, xPosOpponentGroundLine);
    SET_SYMBOL(theFieldDimensions, xPosOpponentGoalPost);
    SET_SYMBOL(theFieldDimensions, xPosOpponentGoal);
    SET_SYMBOL(theFieldDimensions, xPosOpponentFieldBorder);

    SET_SYMBOL(theFieldDimensions, yPosLeftFieldBorder);
    SET_SYMBOL(theFieldDimensions, yPosLeftSideline);
    SET_SYMBOL(theFieldDimensions, yPosLeftPenaltyArea);
    SET_SYMBOL(theFieldDimensions, yPosLeftGoal);
    SET_SYMBOL(theFieldDimensions, yPosLeftGoalArea);
    SET_SYMBOL(theFieldDimensions, yPosRightGoalArea);
    SET_SYMBOL(theFieldDimensions, yPosRightGoal);
    SET_SYMBOL(theFieldDimensions, yPosRightPenaltyArea);
    SET_SYMBOL(theFieldDimensions, yPosRightSideline);
    SET_SYMBOL(theFieldDimensions, yPosRightFieldBorder);
    SET_SYMBOL(theFieldDimensions, yPosCenterGoal);

    SET_SYMBOL(theFieldDimensions, centerCircleRadius);
    SET_SYMBOL(theFieldDimensions, fieldLinesWidth);
    SET_SYMBOL(theFieldDimensions, goalPostRadius);
    SET_SYMBOL(theFieldDimensions, penaltyMarkSize);
    
    SET_SYMBOL(theRobotDimensions, footLength);
    #undef SET_SYMBOL

    // Load strategies.
  FOREACH_ENUM(Strategy::Type, strategy)
  {
    if(strategy == Strategy::none)
      continue;

    InExprMapFile stream(std::string("Behavior/Strategies/") + TypeRegistry::getEnumName(strategy) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> strategies[strategy];

    for(const auto& state : strategies[strategy].tactics)
      for(const auto& transition : state.transitions)
        for(const auto& condition : transition.conditions)
        {
          ballXTimestamps[condition.ballXThreshold] = BallXTimestamps();
          ballYTimestamps[condition.ballYThreshold] = BallYTimestamps();
        }
  }

  // Load tactics.
  FOREACH_ENUM(Tactic::Type, tactic)
  {
    if(tactic == Tactic::none)
      continue;
    InExprMapFile stream(std::string("Behavior/Tactics/") + TypeRegistry::getEnumName(tactic) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> tactics[tactic];
#ifndef NDEBUG
    tactics[tactic].verify(tactic);
#endif
  }

  // Load set plays (and put them into the total set play array).
  setPlays.resize(SetPlay::Type_Info::numOfElements, nullptr);
  FOREACH_ENUM(OwnKickOff::Type, ownKickOff)
  {
    InExprMapFile stream(std::string("Behavior/KickOffs/") + TypeRegistry::getEnumName(ownKickOff) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> ownKickOffs[ownKickOff];
#ifndef NDEBUG
    ownKickOffs[ownKickOff].verify(OwnKickOff::toSetPlay(ownKickOff), tactics);
#endif
    setPlays[OwnKickOff::toSetPlay(ownKickOff)] = &ownKickOffs[ownKickOff];
  }

  FOREACH_ENUM(OpponentKickOff::Type, opponentKickOff)
  {
    InExprMapFile stream(std::string("Behavior/KickOffs/") + TypeRegistry::getEnumName(opponentKickOff) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> opponentKickOffs[opponentKickOff];
#ifndef NDEBUG
    opponentKickOffs[opponentKickOff].verify(OpponentKickOff::toSetPlay(opponentKickOff), tactics);
#endif
    setPlays[OpponentKickOff::toSetPlay(opponentKickOff)] = &opponentKickOffs[opponentKickOff];
  }

// TODO À remettre lorsqu'on aura les penaltyShoots
  FOREACH_ENUM(OwnPenaltyKick::Type, ownPenaltyKick)
  {
    InExprMapFile stream(std::string("Behavior/PenaltyKicks/") + TypeRegistry::getEnumName(ownPenaltyKick) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> ownPenaltyKicks[ownPenaltyKick];
#ifndef NDEBUG
    ownPenaltyKicks[ownPenaltyKick].verify(OwnPenaltyKick::toSetPlay(ownPenaltyKick), tactics);
#endif
    setPlays[OwnPenaltyKick::toSetPlay(ownPenaltyKick)] = &ownPenaltyKicks[ownPenaltyKick];
  }

  FOREACH_ENUM(OpponentPenaltyKick::Type, opponentPenaltyKick)
  {
    InExprMapFile stream(std::string("Behavior/PenaltyKicks/") + TypeRegistry::getEnumName(opponentPenaltyKick) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> opponentPenaltyKicks[opponentPenaltyKick];
#ifndef NDEBUG
    opponentPenaltyKicks[opponentPenaltyKick].verify(OpponentPenaltyKick::toSetPlay(opponentPenaltyKick), tactics);
#endif
    setPlays[OpponentPenaltyKick::toSetPlay(opponentPenaltyKick)] = &opponentPenaltyKicks[opponentPenaltyKick];
  }
  FOREACH_ENUM(OwnFreeKick::Type, ownFreeKick)
  {
    InExprMapFile stream(std::string("Behavior/FreeKicks/") + TypeRegistry::getEnumName(ownFreeKick) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> ownFreeKicks[ownFreeKick];
#ifndef NDEBUG
    ownFreeKicks[ownFreeKick].verify(OwnFreeKick::toSetPlay(ownFreeKick), tactics);
#endif
    setPlays[OwnFreeKick::toSetPlay(ownFreeKick)] = &ownFreeKicks[ownFreeKick];
  }
  FOREACH_ENUM(OpponentFreeKick::Type, opponentFreeKick)
  {
    InExprMapFile stream(std::string("Behavior/FreeKicks/") + TypeRegistry::getEnumName(opponentFreeKick) + ".cfg", symbols, ~bit(InMap::missingAttribute));
    ASSERT(stream.exists());
    stream >> opponentFreeKicks[opponentFreeKick];
#ifndef NDEBUG
    opponentFreeKicks[opponentFreeKick].verify(OpponentFreeKick::toSetPlay(opponentFreeKick), tactics);
#endif
    setPlays[OpponentFreeKick::toSetPlay(opponentFreeKick)] = &opponentFreeKicks[opponentFreeKick];
  }
  for(SetPlay* setPlay : setPlays)
  {
    if(!setPlay)
      continue;
    setPlay->compileVoronoiRegions(tactics);
  }
}

void TacticProvider::update(CurrentTactic& theCurrentTactic)
{
	// Update the ball x timestamps.
  updateBallXTimestamps();
  // Update the ball y timestamps.
  updateBallYTimestamps();
  
  // Update the tactic with the tactic state machine.
  auto checkTransitionConditions = [&](const Strategy::TacticState::Transition::Condition& condition)
  {
    // Count the number of field players. (need to see if we will use agents there like in BHuman code)
    std::size_t numOfFieldPlayers = std::count_if(theTeamData.teammates.begin(), theTeamData.teammates.end(), [](auto& agent){return !agent.isGoalkeeper && !agent.isPenalized;});
		if (!theRobotInfo.isGoalkeeper()) numOfFieldPlayers++;
		
    const int scoreDifference = theOwnTeamInfo.score - theOpponentTeamInfo.score;
    const auto ballXTimestamp = ballXTimestamps.find(condition.ballXThreshold);
    const auto ballYTimestamp = ballYTimestamps.find(condition.ballYThreshold);
    return condition.numOfFieldPlayersLE >= numOfFieldPlayers &&
            condition.numOfFieldPlayersGE <= numOfFieldPlayers &&
            condition.ownScoreLE >= theOwnTeamInfo.score &&
            condition.ownScoreGE <= theOwnTeamInfo.score &&
            condition.opponentScoreLE >= theOpponentTeamInfo.score &&
            condition.opponentScoreGE <= theOpponentTeamInfo.score &&
            condition.scoreDifferenceLE >= scoreDifference &&
            condition.scoreDifferenceGE <= scoreDifference &&
            condition.timeSinceBallAheadOfThresholdGE <= theFrameInfo.getTimeSince(ballXTimestamp->second.lastTimeWhenNotAhead) &&
            condition.timeSinceBallBehindThresholdGE <= theFrameInfo.getTimeSince(ballXTimestamp->second.lastTimeWhenNotBehind)&&
            condition.timeSinceBallLeftOfThresholdGE <= theFrameInfo.getTimeSince(ballYTimestamp->second.lastTimeWhenNotAhead) &&
            condition.timeSinceBallRightOfThresholdGE <= theFrameInfo.getTimeSince(ballYTimestamp->second.lastTimeWhenNotBehind);
  };
  
  FOREACH_ENUM(Strategy::Type, strategy)
  {
	  
    if(strategy == Strategy::none)
      continue;

    SetPlay::Type setPlayType = SetPlay::none;
    // Handle set plays.
    const bool isKickingTeam = theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber;
    if(theGameInfo.state == STATE_READY)
    {
      // During the ready state, all agents can freely propose a set play.
      const SetPlay::GameState setPlayGameState = theGameInfo.setPlay == SET_PLAY_PENALTY_KICK ?
        (isKickingTeam ? SetPlay::ownPenaltyKick : SetPlay::opponentPenaltyKick) :
        (isKickingTeam ? SetPlay::ownKickOff : SetPlay::opponentKickOff);
	  
      switch(setPlayGameState)
      {
        case SetPlay::ownKickOff:
          setPlayType = selectNewSetPlay<OwnKickOff>(strategies[strategy].ownKickOffs, true);
          break;
        case SetPlay::opponentKickOff:
          setPlayType = selectNewSetPlay<OpponentKickOff>(strategies[strategy].opponentKickOffs, true);
          break;
        case SetPlay::ownPenaltyKick:
          setPlayType = selectNewSetPlay<OwnPenaltyKick>(strategies[strategy].ownPenaltyKicks, false);
          break;
        case SetPlay::opponentPenaltyKick:
          setPlayType = selectNewSetPlay<OpponentPenaltyKick>(strategies[strategy].opponentPenaltyKicks, false);
          break;
        default:
          FAIL("Unknown set play type.");
      }
	  
      theCurrentTactic.currentSetPlayType = setPlayType;
      theCurrentTactic.currentSetPlay = *setPlays[setPlayType];
    }else if (theGameInfo.setPlay != SET_PLAY_NONE) {
      if(theGameInfo.gamePhase == SET_PLAY_PENALTY_KICK){
        //TODO handle penalty kick
        if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
          setPlayType = selectNewSetPlay<OwnFreeKick>(strategies[strategy].ownFreeKicks, true);
        }else{
          setPlayType = selectNewSetPlay<OpponentFreeKick>(strategies[strategy].opponentFreeKicks, true);
        }
      }else{
        if(theGameInfo.kickingTeam == theOwnTeamInfo.teamNumber){
          setPlayType = selectNewSetPlay<OwnFreeKick>(strategies[strategy].ownFreeKicks, true);

        }else{
          setPlayType = selectNewSetPlay<OpponentFreeKick>(strategies[strategy].opponentFreeKicks, true);
        }
        theCurrentTactic.currentSetPlayType = setPlayType;
        theCurrentTactic.currentSetPlay = *setPlays[setPlayType];
        theCurrentTactic.currentFreeKick = *static_cast<FreeKick*>(setPlays[setPlayType]);
      }
    }else{
      theCurrentTactic.currentSetPlayType = setPlayType;
    }
    
    // Select a tactic.
    if(setPlayType != SetPlay::none)
    {
      Tactic::Type tacticType = setPlays[setPlayType]->tactic;
      theCurrentTactic.currentTactic = tactics[tacticType];
    }
    else
    {
			for(const Strategy::TacticState& state : strategies[strategy].tactics)
      {
        if (theCurrentTactic.currentTacticType == Tactic::Type::none)
        {
          theCurrentTactic.currentTacticType = state.tactic;
          theCurrentTactic.currentTactic = tactics[state.tactic];
          break;
        }


        if (state.tactic == theCurrentTactic.currentTacticType)
        {
          for(const Strategy::TacticState::Transition& transition : state.transitions)
            if(std::any_of(transition.conditions.begin(), transition.conditions.end(), std::bind(checkTransitionConditions, std::placeholders::_1)))
            {
              theCurrentTactic.currentTacticType = transition.to;
              theCurrentTactic.currentTactic = tactics[transition.to];
              break;
            }
        }
      }
    }
  }
}

void TacticProvider::updateBallXTimestamps()
{
  if(theFieldBall.teamBallWasSeen(8000) || theFieldBall.ballWasSeen(8000))
  {
    const float ballX = theFieldBall.recentBallPositionOnField(8000).x();
    for(auto& ballXTimestamp : ballXTimestamps)
    {
      if(ballX < ballXTimestamp.first)
        ballXTimestamp.second.lastTimeWhenNotAhead = theFrameInfo.time;
      if(ballX > ballXTimestamp.first)
        ballXTimestamp.second.lastTimeWhenNotBehind = theFrameInfo.time;
    }
  }
  else
    for(auto& ballXTimestamp : ballXTimestamps)
      ballXTimestamp.second.lastTimeWhenNotAhead = ballXTimestamp.second.lastTimeWhenNotBehind = theFrameInfo.time;
}

void TacticProvider::updateBallYTimestamps()
{
  if(theFieldBall.teamBallWasSeen(8000) || theFieldBall.ballWasSeen(8000))
  {
    const float ballY = theFieldBall.recentBallPositionOnField(8000).y();
    for(auto& ballYTimestamp : ballYTimestamps)
    {
      if(ballY < ballYTimestamp.first)
        ballYTimestamp.second.lastTimeWhenNotAhead = theFrameInfo.time;
      if(ballY > ballYTimestamp.first)
        ballYTimestamp.second.lastTimeWhenNotBehind = theFrameInfo.time;
    }
  }
  else
    for(auto& ballYTimestamp : ballYTimestamps)
      ballYTimestamp.second.lastTimeWhenNotAhead = ballYTimestamp.second.lastTimeWhenNotBehind = theFrameInfo.time;
}

bool TacticProvider::checkSetPlayStartConditions(SetPlay::Type setPlay) const
{
  
  if (SetPlay::isPenaltyKick(setPlay) || SetPlay::isKickOff(setPlay))
  {
    // No need to implement something here as long as there is only one penalty kick.
    return true;
  }
  else if (SetPlay::isFreeKick(setPlay))
  {
    const auto& freeKick = *static_cast<const FreeKick*>(setPlays[setPlay]);
    const FreeKick::Condition& condition = freeKick.preconditions;
    const bool useBallDropInModel = !(theFieldBall.teamBallWasSeen() || theFieldBall.ballWasSeen(8000)) && theBallDropInModel.isValid && !theBallDropInModel.dropInPositions.empty();
    // It doesn't matter which drop in position is used because the sign of the y coordinate is not important.
    const Vector2f ballPosition = useBallDropInModel ? theBallDropInModel.dropInPositions[0] : theFieldBall.recentBallPositionOnField(8000);
    const Vector2f opponentGoal(theFieldDimensions.xPosOpponentGroundLine, 0.f);
    const float ballToOpponentGoalDistance = (opponentGoal - ballPosition).norm();
    const Angle ballToOpponentGoalAbsAngle = std::abs((opponentGoal - ballPosition).angle());
    const float ballX = ballPosition.x();
    const float ballAbsY = std::abs(ballPosition.y());
    return condition.ballToOpponentGoalDistanceLE >= ballToOpponentGoalDistance &&
      condition.ballToOpponentGoalDistanceGE <= ballToOpponentGoalDistance &&
      condition.ballToOpponentGoalAbsAngleLE >= ballToOpponentGoalAbsAngle &&
      condition.ballToOpponentGoalAbsAngleGE <= ballToOpponentGoalAbsAngle &&
      condition.ballXLE >= ballX &&
      condition.ballXGE <= ballX &&
      condition.ballYAbsLE >= ballAbsY &&
      condition.ballYAbsGE <= ballAbsY;
  }
  else
    FAIL("Unknown set play type.");
  return true;
}

template<typename SetPlayType>
SetPlay::Type TacticProvider::selectNewSetPlay(const std::vector<Strategy::WeightedSetPlay<SetPlayType>>& setPlays, bool random) const
{
  const std::uint32_t seed = theOwnTeamInfo.score | (theOpponentTeamInfo.score << 4) | (theOpponentTeamInfo.teamNumber << 8) | (Global::getSettings().magicNumber << 16);
  std::mt19937 generator(seed);
  std::vector<const Strategy::WeightedSetPlay<SetPlayType>*> applicableSetPlays;
  float weightSum = 0.f;
  for (const Strategy::WeightedSetPlay<SetPlayType>& weightedSetPlay : setPlays)
  {
    if (checkSetPlayStartConditions(SetPlayType::toSetPlay(weightedSetPlay.type)))
    {
      applicableSetPlays.push_back(&weightedSetPlay);
      weightSum += weightedSetPlay.weight;
    }
  }
  if (applicableSetPlays.empty())
    return SetPlay::none;
  if (weightSum == 0.f)
  {
    std::size_t index = random ? Random::uniformInt(applicableSetPlays.size() - 1) : std::uniform_int_distribution<std::size_t>(0, applicableSetPlays.size() - 1)(generator);
    return SetPlayType::toSetPlay(applicableSetPlays[index]->type);
  }
  const float index = random ? Random::uniform(0.f, weightSum) : std::uniform_real_distribution<float>(0.f, weightSum)(generator);
  float currentSum = 0.f;
  for (const Strategy::WeightedSetPlay<SetPlayType>* weightedSetPlay : applicableSetPlays)
  {
    currentSum += weightedSetPlay->weight;
    if (index < currentSum)
      return SetPlayType::toSetPlay(weightedSetPlay->type);
  }
  return SetPlayType::toSetPlay(applicableSetPlays.back()->type);
}