/**
 * @file SupporterPositioningProvider.cpp
 *
 * This file declares a module that provides the SupporterPositioning representation
 *
 * @author Olivier St-Pierre
 * @author Jérémie Thu-Thon
 */

#include "SupporterPositioningProvider.h"
#include "Tools/BehaviorControl/Strategy/Tactic.h"
#include "Tools/BehaviorControl/Strategy/SetPlay.h"
#include "Tools/BehaviorControl/Strategy/FreeKick.h"

#include "Tools/Math/Pose2f.h"

#include <iostream>
using namespace std;

MAKE_MODULE(SupporterPositioningProvider, behaviorControl);

void SupporterPositioningProvider::update(SupporterPositioning& theSupporterPositioning)
{
  if(theRobotInfo.penalty != PENALTY_NONE)
    return;

  DECLARE_DEBUG_DRAWING("behavior:activeRole", "drawingOnField");
  DECLARE_DEBUG_DRAWING("behavior:Tactic:voronoiDiagram", "drawingOnField");

  if (theGameInfo.state != STATE_PLAYING && theGameInfo.state != STATE_READY) return;

  bool dontChangePositions = true;

  const std::vector<std::vector<std::vector<Tactic::Position::Type>>>* positionSubsets = &theCurrentTactic.currentTactic.positionSubsetsPerNumOfAgents;
  const std::vector<std::vector<std::vector<std::vector<Vector2f>>>>* voronoiRegionSubsets = &theCurrentTactic.currentTactic.voronoiRegionSubsetsPerNumOfAgents;

  ASSERT(positionSubsets->size() == voronoiRegionSubsets->size());
  auto positions = theCurrentTactic.currentTactic.positions;

  // Integrate positions from set plays.
  if(theCurrentTactic.currentSetPlayType != SetPlay::none)
  {
    const SetPlay& sP = theCurrentTactic.currentSetPlay;
    positionSubsets = &sP.positionSubsetsPerNumOfAgents;
    voronoiRegionSubsets = &sP.voronoiRegionSubsetsPerNumOfAgents;
    for(Tactic::Position& position : positions)
    for(const SetPlay::Position& positionOverride : sP.positions)
      if(positionOverride.position == position.type)
      {
        position.pose = positionOverride.pose;
        break;
      }
  }

  if (theGameInfo.state == STATE_READY && IS_PREGAME_STATE(theExtendedGameInfo.gameStateBeforeCurrent)) {
    Tactic::Position position = firstReadyAssignPosition(positions);

    theSupporterPositioning.position = position.type;
    theSupporterPositioning.basePose = position.pose;

    return;
  }

  // Initialize a set of pointers to the agents that haven't been assigned a position yet.
  std::vector<Agent> remainingAgents;
  for(const Teammate& teammate : theTeamData.teammates){
    if(teammate.theRobotStatus.isPenalized || teammate.isGoalkeeper || teammate.theTeamBehaviorStatus.role.role == PlayerRole::none){
      continue;
    }
    Agent agent;
    agent.number = teammate.number;
    agent.positionOnField = teammate.theRobotPose.translation;
    agent.position = teammate.supporterPosition;
    remainingAgents.push_back(agent);
  }
  if(!theRobotInfo.isGoalkeeper()){
      Agent ownAgent;
      ownAgent.number = theRobotInfo.number;
      ownAgent.positionOnField = theRobotPose.translation;
      remainingAgents.push_back(ownAgent);
  }


  // If the tactic contains a goalkeeper position, assign that to the goalkeeper agent (and nobody else).
  for(std::size_t i = 0; i < positions.size(); i++)
  {
    const Tactic::Position& position = positions[i];
    if(position.type == Tactic::Position::goalkeeper)
    {
        if(theRobotInfo.isGoalkeeper())
        {
          theSupporterPositioning.position = position.type;
          theSupporterPositioning.basePose = position.pose;
          theSupporterPositioning.baseArea = {Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosLeftGoalArea),
                            Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosLeftGoalArea),
                            Vector2f(theFieldDimensions.xPosOwnGroundLine, theFieldDimensions.yPosRightGoalArea),
                            Vector2f(theFieldDimensions.xPosOwnGoalArea, theFieldDimensions.yPosRightGoalArea)};

          COMPLEX_DRAWING("behavior:Tactic:voronoiDiagram")
          {
            POLYGON("behavior:Tactic:voronoiDiagram", theSupporterPositioning.baseArea.size(), theSupporterPositioning.baseArea.data(), 40, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 0, 0));
            CIRCLE("behavior:Tactic:voronoiDiagram", theSupporterPositioning.basePose.translation.x(), theSupporterPositioning.basePose.translation.y(), 60, 20, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA::black);
          }
          return;
        }
    }
  }
  if(!remainingAgents.empty())
  {
    ASSERT(remainingAgents.size() <= positionSubsets->size());

    const auto& suitableSubsets = (*positionSubsets)[remainingAgents.size() - 1];
    const auto& suitableVoronoiRegionSubsets = (*voronoiRegionSubsets)[remainingAgents.size() - 1];

    // Sort remaining agents because there is at least one point where the results depend on the order of the rows of the cost matrix (minCoeff in startPositionSpecialHandling).
    // Okay, it's not minCoeff anymore, but I am hesitatant to remove this now. (2023-06-24)
    std::sort(remainingAgents.begin(), remainingAgents.end(), [](Agent a1, Agent a2){ return a1.number < a2.number; });

    // Initialize a set of pointers to the positions that haven't been assigned to an agent yet.
    std::array<const Tactic::Position*, Tactic::Position::numOfTypes> positionMap {nullptr};
    for(const Tactic::Position& position : positions)
      positionMap[position.type] = &position;

    std::array<const Tactic::Position*, Tactic::Position::numOfTypes> remainingPositions {nullptr};
    std::array<const std::vector<Vector2f>*, Tactic::Position::numOfTypes> remainingVoronoiRegions {nullptr};
    ASSERT(suitableSubsets.size() == suitableVoronoiRegionSubsets.size());

    for(size_t i = 0; i < suitableSubsets.size(); i++)
    {
      ASSERT(suitableSubsets[i].size() == suitableVoronoiRegionSubsets[i].size());
      for(size_t j = 0; j < suitableSubsets[i].size(); j++)
      {
        Tactic::Position::Type position = suitableSubsets[i][j];
        remainingPositions[position] = positionMap[position];
        remainingVoronoiRegions[position] = &suitableVoronoiRegionSubsets[i][j];
      }
    }

    // Construct a cost matrix where the rows correspond to agents and the columns correspond to positions.
    // The columns of the matrix alternate between the original position and the mirrored positions.
    // However, columns representing a mirrored position are only calculated for positions which can actually be mirrored.
    Eigen::MatrixXf costMatrix(remainingAgents.size(), 2 * remainingPositions.size());
    // Note here than Eigen matrices are stored column-major.
    for(Eigen::MatrixXf::Index positionIndex = 0; positionIndex < costMatrix.cols(); ++positionIndex)
    {
      const Tactic::Position* p = remainingPositions[positionIndex / 2];
      if(!p)
        continue;
      const bool mirror = positionIndex % 2;
      // If multiple agents claim to have had the same position, only the one with the lowest number counts.
      int theOneAndOnlyAgent = Settings::highestValidPlayerNumber + 1;
      if(theGameInfo.state != STATE_READY && theGameInfo.state != STATE_SET)
        for(Agent& agent : remainingAgents)
          if(agent.position == Tactic::Position::mirrorIf(p->type, mirror) && agent.number < theOneAndOnlyAgent)
            theOneAndOnlyAgent = agent.number;
      const Vector2f target(p->pose.translation.x(), mirror ? -p->pose.translation.y() : p->pose.translation.y());
      for(Eigen::MatrixXf::Index agentIndex = 0; agentIndex < costMatrix.rows(); ++agentIndex)
      {
        costMatrix(agentIndex, positionIndex) = (target - remainingAgents[agentIndex].positionOnField).norm();
        if(remainingAgents[agentIndex].number == theOneAndOnlyAgent)
        {
          if(dontChangePositions)
            costMatrix(agentIndex, positionIndex) = std::numeric_limits<float>::lowest();
          else
            costMatrix(agentIndex, positionIndex) -= p->stabilityOffset;
        }
      }
    }

    std::array<std::vector<std::size_t>, 2> bestAssignment;
    std::array<std::vector<float>, 2> bestAssignmentCost;
    // In some cases, we could already know that one of the mirror variants is not going to be used.
    // But you know what they say about premature optimization...
    for(bool mirrored : {false, true})
    {
      for(const std::vector<Tactic::Position::Type>& subset : suitableSubsets)
      {
        // Construct a sorted version of the position indices (since the first assignment must be the lexicographically smallest one).
        std::vector<std::size_t> initialAssignment(subset.size());
        std::copy(subset.begin(), subset.end(), initialAssignment.begin());
        std::sort(initialAssignment.begin(), initialAssignment.end());

        // Construct the initial index list from the subset, mapping position indices to cost matrix indices.
        std::vector<std::size_t> assignment = initialAssignment;
        for(std::size_t& index : assignment)
        {
          index *= 2;
          if(mirrored)
            ++index;
        }

        // Search through all orderings of the position subset which represent assignments of agents to positions.
        do
        {
          // This is quite an inefficient way of doing this: Rejecting all invalid assignments while it would be possible to avoid generating them at all...
          const std::vector<float> assignmentCost = getAssignmentCost(costMatrix, assignment);
          if(bestAssignmentCost[mirrored].empty() ||
            std::lexicographical_compare(assignmentCost.begin(), assignmentCost.end(), bestAssignmentCost[mirrored].begin(), bestAssignmentCost[mirrored].end()))
          {
            bestAssignmentCost[mirrored] = assignmentCost;
            bestAssignment[mirrored] = assignment;
          }
        }
        while(std::next_permutation(assignment.begin(), assignment.end()));
      }
      // Make sure that there was a valid assignment.
      ASSERT(!bestAssignmentCost[mirrored].empty());
    }
    ASSERT(!bestAssignment[false].empty() || !bestAssignment[true].empty());

    // TODO: walking in from sidelines special handling: prefer assigning startPosition to a player from the left

    // TODO: move this to updateSetPlay
    // TODO: setPlay != SET_PLAY_NONE -> setPlayStep < 0?
    // TODO: don't check team ball model continuously, but only before the free kick starts
    // TODO checker le vote ici
    bool acceptedMirror = false;

    if(SetPlay::isFreeKick(theCurrentTactic.currentSetPlayType) && theCurrentTactic.currentFreeKick.ballSide != FreeKick::irrelevant)
    {
      if((theCurrentTactic.currentFreeKick.ballSide == FreeKick::left && theFieldBall.recentBallPositionOnField().y() < 0.f) ||
         (theCurrentTactic.currentFreeKick.ballSide == FreeKick::right && theFieldBall.recentBallPositionOnField().y() > 0.f))
        acceptedMirror = true;
      else
        acceptedMirror = false;
    }

    for(std::size_t i = 0; i < bestAssignment[acceptedMirror].size(); ++i)
    {
      const size_t positionIndex = bestAssignment[acceptedMirror][i] / 2;
      const Tactic::Position& position = *remainingPositions[positionIndex];

      Tactic::Position::Type positionToSet = Tactic::Position::mirrorIf(position.type, acceptedMirror);
      Pose2f basePose = acceptedMirror ? Pose2f(-position.pose.rotation,
        position.pose.translation.x(),
        -position.pose.translation.y())
        : position.pose;
      if (theRobotInfo.number == remainingAgents[i].number) {
          theSupporterPositioning.position = positionToSet;
          theSupporterPositioning.basePose = basePose;

      }
    DRAW_TEXT("behavior:activeRole", remainingAgents[i].positionOnField.x(), remainingAgents[i].positionOnField.y(), 100, ColorRGBA::red, positionToSet);
    remainingAgents[i].position = positionToSet;
    remainingAgents[i].basePose = basePose;
    assignPosition(positionToSet, remainingAgents[i]);

      if(!remainingVoronoiRegions[positionIndex])
        continue;
      if (theRobotInfo.number == remainingAgents[i].number) {
        theSupporterPositioning.baseArea = *remainingVoronoiRegions[positionIndex];
        remainingAgents[i].baseArea = *remainingVoronoiRegions[positionIndex];
      }
      if (acceptedMirror) {
        if (theRobotInfo.number == remainingAgents[i].number) {
          for(Vector2f& point : theSupporterPositioning.baseArea)
            point = Vector2f(point.x(), -point.y());
        }
      }

    }

    COMPLEX_DRAWING("behavior:Tactic:voronoiDiagram")
    {
      for(std::size_t i = 0; i < bestAssignment[acceptedMirror].size(); ++i)
      {
        POLYGON("behavior:Tactic:voronoiDiagram", remainingAgents[i].baseArea.size(), remainingAgents[i].baseArea.data(), 40, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA(0, 0, 0, 0));
        CIRCLE("behavior:Tactic:voronoiDiagram", remainingAgents[i].basePose.translation.x(), remainingAgents[i].basePose.translation.y(), 60, 20, Drawings::solidPen, ColorRGBA::black, Drawings::solidBrush, ColorRGBA::black);
      }
    }
  }
  //else
    // proposedMirror = acceptedMirror = false;
}

std::vector<float> SupporterPositioningProvider::getAssignmentCost(const Eigen::MatrixXf& costMatrix, const std::vector<std::size_t>& positionIndices)
{
  std::vector<float> result(positionIndices.size());
  ASSERT(positionIndices.size() == static_cast<std::size_t>(costMatrix.rows()));
  for(std::size_t i = 0; i < result.size(); ++i)
    result[i] = costMatrix(i, positionIndices[i]);
  std::sort(result.begin(), result.end(), std::greater<>());
  return result;
}

void SupporterPositioningProvider::assignPosition(Tactic::Position::Type positionToSet, Agent& agent)
{
  std::vector<Teammate> teammates = theTeamData.teammates;
  for (int i = 0; i < (int)teammates.size(); i++) {
    if (agent.number == teammates[i].number) {
      teammates[i].supporterPosition = positionToSet;
    }
  }
}

Tactic::Position SupporterPositioningProvider::firstReadyAssignPosition(std::vector<Tactic::Position> positions)
{
  for(std::size_t i = 0; i < positions.size(); i++)
  {
    const Tactic::Position& position = positions[i];

    switch (theRobotInfo.number)
    {
      case 1:
        if (position.type == Tactic::Position::Type::goalkeeper) {
          return position;
        }
        break;
      case 2:
        if (position.type == Tactic::Position::Type::defenderR) {
          return position;
        }
        break;
      case 3:
        if (position.type == Tactic::Position::Type::defenderL) {
          return position;
        }
        break;
      case 4:
        if (position.type == Tactic::Position::Type::midfielder) {
          return position;
        }
        break;
      case 5:
        if (position.type == Tactic::Position::Type::forward) {
          return position;
        }
        break;
      case SUB_NUMBER:
        if (position.type == Tactic::Position::Type::none) {
          return position;
        }
        break;
      default:
        FAIL("Number of robot is not good");
        return positions[0]; //Should never be called (Fix for compiler warning)
    }
  }
  FAIL("No robots");
  return positions[0]; //Should never be called (Fix for compiler warning)
}
