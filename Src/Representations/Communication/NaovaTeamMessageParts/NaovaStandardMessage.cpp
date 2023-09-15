#include "NaovaStandardMessage.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include <cassert>
#include <algorithm>

template <typename T>
inline void writeVal(void *&data, T value)
{
  *reinterpret_cast<T *&>(data)++ = value;
}

template <typename T>
inline T readVal(const void *&data)
{
  return *reinterpret_cast<T *&>(data)++;
}

namespace Naova
{
  int Obstacle::sizeOfObstacle()
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");
    return 5;
  }

  void Obstacle::write(void *&data, uint32_t timestamp) const
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "The constructor is not adjusted for the current message version");
#ifndef NDEBUG
    const void *const begin = data; // just for length check
#endif                              // NDEBUG
                                    // write data (5 bytes)
    writeVal<int16_t>(data, static_cast<int16_t>(((static_cast<int16_t>(center[0]) >> 2) & 0x3FFF) | ((static_cast<int>(type) & 0xC) << 12)));
    writeVal<int16_t>(data, static_cast<int16_t>(((static_cast<int16_t>(center[1]) >> 2) & 0x3FFF) | ((static_cast<int>(type) & 0x3) << 14)));

    const uint32_t timestampLastSeenDiff64 = (timestamp - timestampLastSeen) >> 6;
    writeVal<uint8_t>(data, static_cast<uint8_t>(timestampLastSeenDiff64 > 0xFE ? 0xFF : timestampLastSeenDiff64));

    assert((reinterpret_cast<char *>(data) - reinterpret_cast<const char *const>(begin)) == sizeOfObstacle());
  }

  void Obstacle::read(const void *&data, uint32_t timestamp)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "The constructor is not adjusted for the current message version");
    const int16_t center0Struct = readVal<const int16_t>(data);
    const int16_t center1Struct = readVal<const int16_t>(data);

    center[0] = static_cast<float>(static_cast<int16_t>(center0Struct << 2));
    center[1] = static_cast<float>(static_cast<int16_t>(center1Struct << 2));

    type = static_cast<ObstacleType>((static_cast<uint16_t>(center0Struct & 0xC000) >> 12) | (static_cast<uint16_t>(center1Struct & 0xC000) >> 14));

    const uint8_t timestampLastSeenDiff64 = readVal<const uint8_t>(data);
    timestampLastSeen = timestamp - (static_cast<uint32_t>(timestampLastSeenDiff64) << 6);
  }

  OwnTeamInfo::OwnTeamInfo() : timestampWhenReceived(0) {}

  int OwnTeamInfo::sizeOfOwnTeamInfo() const
  {
    return +1                         // timestampWhenReceived
           + sizeof(packetNumber) + 1 // gameType, state firstHalf, gamePhase
           + 1                        // kickingTeam or dropInTeam
           + 3                        // dropInTime, secRemaining, secondaryTime
           + 2                        // score, playersArePenalized
        ;
  }

  void OwnTeamInfo::write(void *&data, uint32_t timestamp) const
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "The constructor is not adjusted for the current message version");

#ifndef NDEBUG
    const void *const begin = data; // just for length check
#endif                              // NDEBUG

    writeVal<uint8_t>(data, static_cast<uint8_t>((std::min(static_cast<unsigned>(0xFFFF), timestamp - timestampWhenReceived) >> 8) & 0xFF));
    writeVal<uint8_t>(data, packetNumber);

    writeVal<uint8_t>(data, static_cast<uint8_t>((competitionType & 3) << 6 | (state & 7) << 3 | (firstHalf & 1) << 2 | (gamePhase & 3)));
    writeVal<uint8_t>(data, state == STATE_PLAYING ? dropInTeam : kickingTeam);

    writeVal<uint16_t>(data, static_cast<uint16_t>(std::min(63u, 1u + dropInTime) >> 1 << 11 | (secsRemaining & 0x3FF) << 1 | (secondaryTime >> 8 & 1)));
    writeVal<uint8_t>(data, static_cast<uint8_t>(secondaryTime & 0xFF));

    uint16_t scorePlusIsPenalized = score;
    for (unsigned i = 0; i < NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
      (scorePlusIsPenalized <<= 1) |= playersArePenalized[i] ? 1 : 0;
    writeVal<uint16_t>(data, scorePlusIsPenalized);

    assert((reinterpret_cast<char *>(data) - reinterpret_cast<const char *const>(begin)) == sizeOfOwnTeamInfo());
  }

  void OwnTeamInfo::read(const void *&data, uint32_t timestamp)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "The constructor is not adjusted for the current message version");

    timestampWhenReceived = timestamp - (readVal<const uint8_t>(data) << 8);
    if (timestamp - timestampWhenReceived == 0xFF00)
      timestampWhenReceived = 0;

    packetNumber = readVal<const int8_t>(data);

    const uint8_t stateStruct = readVal<const int8_t>(data);
    competitionType = stateStruct >> 6;
    competitionPhase = stateStruct >> 6;
    state = (stateStruct >> 3) & 7;
    firstHalf = (stateStruct >> 2) & 1;
    gamePhase = stateStruct & 3;

    if (state == STATE_PLAYING)
      dropInTeam = readVal<const int8_t>(data);
    else
      kickingTeam = readVal<const int8_t>(data);

    const uint16_t timeStruct = readVal<const int16_t>(data);
    dropInTime = static_cast<uint16_t>(timeStruct >> 11 << 1);
    secsRemaining = static_cast<uint16_t>((timeStruct >> 1) & 0x3FF);
    secondaryTime = static_cast<uint16_t>((stateStruct & 1) << 8);
    secondaryTime |= readVal<const int8_t>(data);

    const uint16_t scorePenaltyStruct = readVal<const int16_t>(data);
    score = (uint8_t)(scorePenaltyStruct >> NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS);
    unsigned runner = 1 << (NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS - 1);
    for (unsigned i = 0; i < NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, runner >>= 1)
      playersArePenalized[i] = (runner & scorePenaltyStruct) != 0;
  }

  Out &operator<<(Out &stream, const Obstacle &bHSTMObstacle)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(bHSTMObstacle);
    STREAM_EXT(stream, bHSTMObstacle.center);
    STREAM_EXT(stream, bHSTMObstacle.timestampLastSeen);
    STREAM_EXT(stream, bHSTMObstacle.type, Naova);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  In &operator>>(In &stream, Obstacle &bHSTMObstacle)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(bHSTMObstacle);
    STREAM_EXT(stream, bHSTMObstacle.center);
    STREAM_EXT(stream, bHSTMObstacle.timestampLastSeen);
    STREAM_EXT(stream, bHSTMObstacle.type, Naova);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  Out &operator<<(Out &stream, const BNTPMessage &ntpMessage)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(ntpMessage);
    STREAM_EXT(stream, ntpMessage.receiver);
    STREAM_EXT(stream, ntpMessage.requestOrigination);
    STREAM_EXT(stream, ntpMessage.requestReceipt);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  In &operator>>(In &stream, BNTPMessage &ntpMessage)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(ntpMessage);
    STREAM_EXT(stream, ntpMessage.receiver);
    STREAM_EXT(stream, ntpMessage.requestOrigination);
    STREAM_EXT(stream, ntpMessage.requestReceipt);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  Out &operator<<(Out &stream, const OwnTeamInfo &ownTeamInfo)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(ownTeamInfo);
    STREAM_EXT(stream, ownTeamInfo.timestampWhenReceived);
    STREAM_EXT(stream, ownTeamInfo.packetNumber);
    STREAM_EXT(stream, ownTeamInfo.competitionType);
    STREAM_EXT(stream, ownTeamInfo.competitionPhase);
    STREAM_EXT(stream, ownTeamInfo.state);
    STREAM_EXT(stream, ownTeamInfo.firstHalf);
    STREAM_EXT(stream, ownTeamInfo.kickingTeam);
    STREAM_EXT(stream, ownTeamInfo.gamePhase);
    STREAM_EXT(stream, ownTeamInfo.dropInTeam);
    STREAM_EXT(stream, ownTeamInfo.dropInTime);
    STREAM_EXT(stream, ownTeamInfo.secsRemaining);
    STREAM_EXT(stream, ownTeamInfo.secondaryTime);
    STREAM_EXT(stream, ownTeamInfo.score);
    STREAM_EXT(stream, ownTeamInfo.playersArePenalized);
    STREAM_REGISTER_FINISH;

    return stream;
  }

  In &operator>>(In &stream, OwnTeamInfo &ownTeamInfo)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

    STREAM_REGISTER_BEGIN_EXT(ownTeamInfo);
    STREAM_EXT(stream, ownTeamInfo.timestampWhenReceived);
    STREAM_EXT(stream, ownTeamInfo.packetNumber);
    STREAM_EXT(stream, ownTeamInfo.competitionType);
    STREAM_EXT(stream, ownTeamInfo.competitionPhase);
    STREAM_EXT(stream, ownTeamInfo.state);
    STREAM_EXT(stream, ownTeamInfo.firstHalf);
    STREAM_EXT(stream, ownTeamInfo.kickingTeam);
    STREAM_EXT(stream, ownTeamInfo.gamePhase);
    STREAM_EXT(stream, ownTeamInfo.dropInTeam);
    STREAM_EXT(stream, ownTeamInfo.dropInTime);
    STREAM_EXT(stream, ownTeamInfo.secsRemaining);
    STREAM_EXT(stream, ownTeamInfo.secondaryTime);
    STREAM_EXT(stream, ownTeamInfo.score);
    STREAM_EXT(stream, ownTeamInfo.playersArePenalized);
    STREAM_REGISTER_FINISH;

    return stream;
  }
  
  static const char *getName(HearingConfidence e)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES == 7, "This method is not adjusted for the current message version");

    if (e == HearingConfidence::iAmDeaf || e == HearingConfidence(0))
      return "iAmDeaf";

    if (e == HearingConfidence::heardOnOneEarButThinkingBothAreOk || e == HearingConfidence(1))
      return "heardOnOneEarButThinkingBothAreOk";

    if (e == HearingConfidence::oneEarIsBroken || e == HearingConfidence(2))
      return "oneEarIsBroken";

    if (e == HearingConfidence::allEarsAreOk || e == HearingConfidence(3))
      return "allEarsAreOk";

    return nullptr;
  }

  static const char *getName(ObstacleType e)
  {
    static_assert(NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES == 7, "This method is not adjusted for the current message version");

    if (e >= numOfObstacleTypes)
      return nullptr;

    static const char *names[static_cast<unsigned>(numOfObstacleTypes)] =
        {
            "goalpost",
            "unknown",
            "someRobot",
            "opponent",
            "teammate",
            "fallenSomeRobot",
            "fallenOpponent",
            "fallenTeammate"};
    return names[static_cast<unsigned>(e)];
  }
}
void NaovaStandardMessage::write(void *data) const
{
  static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

  writeVal<uint8_t>(data, magicNumber);


  writeVal<int16_t>(data, ballLastPerceptX);
  writeVal<int16_t>(data, ballLastPerceptY);

  writeVal<std::array<float, 3>>(data, ballCovariance);
  writeVal<float>(data, robotPoseDeviation);

  writeVal<uint8_t>(data, robotPoseValidity);

  writeVal<uint32_t>(data, timestamp);

  writeVal<uint32_t>(data, timestampLastJumped);

  writeVal<uint32_t>(data, ballTimeWhenLastSeen);

  const uint16_t confidenceOfLastWhistleDetectionConverted =
      (confidenceOfLastWhistleDetection == Naova::HearingConfidence::iAmDeaf ? 0
                                                                             : (confidenceOfLastWhistleDetection == Naova::HearingConfidence::heardOnOneEarButThinkingBothAreOk ? 1
                                                                                                                                                                                : (confidenceOfLastWhistleDetection == Naova::HearingConfidence::oneEarIsBroken ? 2
                                                                                                                                                                                                                                                                : 3)));
  const uint16_t lastTimeWhistleDetectedDiff = static_cast<uint16_t>(std::min(timestamp - lastTimeWhistleDetected, 0x3FFFu));
  writeVal<uint16_t>(data, static_cast<uint16_t>(confidenceOfLastWhistleDetectionConverted << 14) | lastTimeWhistleDetectedDiff);

  writeVal<uint8_t>(data, currentlyPerformingRole);


  uint8_t boolContainer = 0;
  (boolContainer) |= isPenalized ? 1 : 0;
  (boolContainer <<= 1) |= isUpright ? 1 : 0;
  (boolContainer <<= 1) |= hasGroundContact ? 1 : 0;

  writeVal<uint8_t>(data, boolContainer);

  writeVal<float>(data, ballVel[0]);
  writeVal<float>(data, ballVel[1]);
  // std::sort(const_cast<std::vector<Naova::BNTPMessage> &>(ntpMessages).begin(), const_cast<std::vector<Naova::BNTPMessage> &>(ntpMessages).end(), [&](const Naova::BNTPMessage &a, const Naova::BNTPMessage &b)
  //           { return a.receiver < b.receiver; });
  // uint16_t ntpReceivers = 0;
  // if (!ntpMessages.empty())
  // {
  //   auto ntpMessagesItr = ntpMessages.cbegin();
  //   for (unsigned i = 0; i < NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, ntpReceivers <<= 1)
  //     if (ntpMessagesItr == ntpMessages.cend())
  //       continue;
  //     else if (ntpMessagesItr->receiver == i + 1)
  //     {
  //       ntpReceivers |= 1;
  //       ntpMessagesItr++;
  //     }
  // }
  // writeVal<uint16_t>(data, static_cast<uint16_t>((boolContainer << 11) | (ntpReceivers >> 1)));
  // for (const Naova::BNTPMessage &ntpMessage : ntpMessages)
  // {
  //   uint32_t requestReceiptDiffCutted = std::min(timestamp - ntpMessage.requestReceipt, 0xFFFu);
  //   writeVal<uint32_t>(data, ntpMessage.requestOrigination | ((requestReceiptDiffCutted & 0xF00u) << 20));
  //   writeVal<uint8_t>(data, static_cast<uint8_t>(requestReceiptDiffCutted & 0xFF));
  // }
}

bool NaovaStandardMessage::read(const void *data)
{
  static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");
#ifndef NDEBUG
  const void *const begin = data; // just for length check
#endif                            // NDEBUG

  magicNumber = readVal<const uint8_t>(data);
  if (!(Global::settingsExist() && Global::getSettings().magicNumber))
    return false;

  ballLastPerceptX = readVal<const int16_t>(data);
  ballLastPerceptY = readVal<const int16_t>(data);

  ballCovariance = readVal<const std::array<float, 3>>(data);
  robotPoseDeviation = readVal<const float>(data);

  // robotPoseCovariance = readVal<const std::array<float, 6>>(data);
  robotPoseValidity = readVal<const uint8_t>(data);

  // obstacles.clear();
  // ntpMessages.clear();

  timestamp = readVal<const uint32_t>(data);

  timestampLastJumped = readVal<const uint32_t>(data);

  // timeWhenReachBall = readVal<const uint32_t>(data);
  // timeWhenReachBallStriker = readVal<const uint32_t>(data);

  ballTimeWhenLastSeen = readVal<const uint32_t>(data);

  // static_assert(Naova::numOfHearingConfidences == Naova::HearingConfidence(4), "Following does not work for that many HearingConfidences. Adjust it!");
  
  const uint16_t whistleDetectionContainer = readVal<const uint16_t>(data);
  confidenceOfLastWhistleDetection = Naova::HearingConfidence(whistleDetectionContainer >> 14);

  if ((whistleDetectionContainer & 0x3FFF) > 0x3FFCu)
    lastTimeWhistleDetected = 0;
  else
    lastTimeWhistleDetected = timestamp - (whistleDetectionContainer & 0x3FFF);

  // gameControlData.read(data, timestamp);

  currentlyPerformingRole = static_cast<Role::RoleType>(readVal<const uint8_t>(data));

  // static_assert(NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 5, "Following does not work for that many teammates. Adjust it!");
  // unsigned runner = 0x7 << ((NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS - 1) * 3);
  
  // for (unsigned i = 0; i < NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i, runner >>= 3)
  //   roleAssignments[i] = static_cast<Role::RoleType>((roleContainer & runner) >> ((NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS - i - 1) * 3));

  // static_assert(NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES < 9, "Following does not work for that many obstacles. Adjust it!");
  // static_assert(NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS < 10, "Following does not work for that many teammates. Adjust it!");
  // const uint8_t numObsCurrPerforRolePassTargContainer = readVal<const uint8_t>(data);
  // passTarget = numObsCurrPerforRolePassTargContainer >> 4;
  // currentlyPerformingRole = static_cast<Role::RoleType>(((numObsCurrPerforRolePassTargContainer & 8) >> 1) | (roleContainer >> ((NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS + 1) * 3)));
  // obstacles.resize(numObsCurrPerforRolePassTargContainer & 0x7u);
  // for (auto &obstacle : obstacles)
  //   obstacle.read(data, timestamp);
  
   uint8_t boolAndNTPReceiptContainer = readVal<const uint8_t>(data);
  // runner = 1 << 15;
  // keeperIsPlaying = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
  // requestsNTPMessage = (boolAndNTPReceiptContainer & (runner >>= 1)) != 0;
  hasGroundContact = boolAndNTPReceiptContainer & 1;
  isUpright = (boolAndNTPReceiptContainer >>= 1) & 1;
  isPenalized = (boolAndNTPReceiptContainer >>= 1) & 1;
  // static_assert(NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS == 5, "Following does not work for that many teammates. Adjust it!");
  // for (uint8_t i = 1; runner != 0; ++i)
  //   if (boolAndNTPReceiptContainer & (runner >>= 1))
  //   {
  //     ntpMessages.emplace_back();
  //     Naova::BNTPMessage &message = ntpMessages.back();
  //     message.receiver = i;

  //     const uint32_t timeStruct32 = readVal<const uint32_t>(data);
  //     const uint8_t timeStruct8 = readVal<const uint8_t>(data);

  //     message.requestOrigination = timeStruct32 & 0xFFFFFFF;
  //     message.requestReceipt = timestamp - ((timeStruct32 >> 20) & 0xF00u) | static_cast<uint32_t>(timeStruct8);
  //   }

  // assert((reinterpret_cast<const char *>(data) - reinterpret_cast<const char *const>(begin)) == sizeOfNaovaMessage());

  ballVel[0] = readVal<const float>(data);
  ballVel[1] = readVal<const float>(data);
  return true;
}
int NaovaStandardMessage::sizeOfNaovaMessage() const
{
  static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

      // return sizeof(header)
      //         + sizeof(version)
          return sizeof(magicNumber)
              // + sizeof(ballTimeWhenDisappearedSeenPercentage)
              + sizeof(ballLastPerceptX)
              + sizeof(ballLastPerceptY)
              + sizeof(ballCovariance)
              + sizeof(robotPoseDeviation)
              // + sizeof(robotPoseCovariance)
              + sizeof(robotPoseValidity)
              + sizeof(timestamp)
              + sizeof(currentlyPerformingRole)
              // + sizeof(timeWhenReachBall)
              // + sizeof(timeWhenReachBallStriker)
              + sizeof(timestampLastJumped)
              + sizeof(ballTimeWhenLastSeen)
              + sizeof(ballVel)
              + 1 // booleanContainer
              // // + gameControlData.sizeOfOwnTeamInfo()
              // + 4 // roleAssignments
              // // + sizeof(keeperIsPlaying)
              // // + sizeof(passTarget)
              // + 2 // timeWhenReachBall
              // + 2 // timeWhenReachBallStriker
              // + 1 // timestampLastJumped
              // + 4; // ballTimeWhenLastSeen
              + 2; // whistle stuff
              // // + std::min(int(obstacles.size()), NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_OBSTACLES) * Naova::Obstacle::sizeOfObstacle()
              // // + sizeof(requestsNTPMessage)
              // // + static_cast<int>(ntpMessages.size()) * 5;
  }

void NaovaStandardMessage::serialize(In *in, Out *out)
{
  static_assert(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION == 2, "This method is not adjusted for the current message version");

  STREAM_REGISTER_BEGIN;
  // std::string headerRef(header, 4);
  // STREAM(headerRef); // does not allow to change the header in any case, but makes it visble in a great way
  // STREAM(version);
  // STREAM(ballTimeWhenDisappearedSeenPercentage);
  STREAM(ballLastPerceptX);
  STREAM(ballLastPerceptY);
  STREAM(ballCovariance);
  STREAM(robotPoseDeviation);
  // STREAM(robotPoseCovariance);
  STREAM(robotPoseValidity);
  STREAM(timestamp);
  // STREAM(gameControlData)
  STREAM(currentlyPerformingRole, Role);
  // STREAM(roleAssignments, Role);
  // STREAM(keeperIsPlaying);
  // STREAM(passTarget);
  // STREAM(timeWhenReachBall);
  // STREAM(timeWhenReachBallStriker);
  STREAM(timestampLastJumped);
  STREAM(ballTimeWhenLastSeen);
  STREAM(confidenceOfLastWhistleDetection, Naova);
  STREAM(lastTimeWhistleDetected);
  STREAM(ballVel);
  // STREAM(obstacles);
  // STREAM(requestsNTPMessage);
  // STREAM(ntpMessages);
  STREAM_REGISTER_FINISH;
}

NaovaStandardMessage::NaovaStandardMessage() : 
  // version(NAOVA_STANDARD_MESSAGE_STRUCT_VERSION),
  // ballTimeWhenDisappearedSeenPercentage(0u),
  isPenalized(true),
  isUpright(true),
  hasGroundContact(true),
  timestamp(0),
  // gameControlData(),
  currentlyPerformingRole(Role::undefined),
  // keeperIsPlaying(false),
  // passTarget(-1),
  // timeWhenReachBall(std::numeric_limits<uint32_t>::max()),
  // timeWhenReachBallStriker(std::numeric_limits<uint32_t>::max()),
  timestampLastJumped(0),
  ballTimeWhenLastSeen(0),
  confidenceOfLastWhistleDetection(Naova::HearingConfidence::iAmDeaf),
  lastTimeWhistleDetected(0)
  // obstacles(),
  // requestsNTPMessage(false),
{
  // const char *init = NAOVA_STANDARD_MESSAGE_STRUCT_HEADER;
  // for (unsigned int i = 0; i < sizeof(header); ++i)
  //   header[i] = init[i];

  // roleAssignments[0] = Role::keeper;
  // for (int i = 1; i < NAOVA_STANDARD_MESSAGE_MAX_NUM_OF_PLAYERS; ++i)
  //   roleAssignments[i] = Role::striker;
  ballVel[0] = 0.f;
  ballVel[1] = 0.f;
};