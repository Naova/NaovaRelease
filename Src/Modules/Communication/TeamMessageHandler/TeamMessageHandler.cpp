/**
 * @file TeamMessageHandler.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "TeamMessageHandler.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Tools/Settings.h"
#include "Platform/File.h"
#include "Platform/Time.h"

//#define SITTING_TEST
//#define SELF_TEST

MAKE_MODULE(TeamMessageHandler, communication);

// BNTP, RobotStatus, RobotPose, FieldCoverage, RobotHealth and FieldFeatureOverview cannot be part of this for technical reasons.
#define FOREACH_TEAM_MESSAGE_REPRESENTATION(_) \
  _(FrameInfo); \
  _(BallModel); \
  _(ObstacleModel); \
  _(Whistle); \
  _(BehaviorStatus); \
  _(TeamBehaviorStatus); \
  _(TeamTalk);

struct TeamMessage
{};

void TeamMessageHandler::regTeamMessage()
{
  PUBLISH(regTeamMessage);
  const char* name = typeid(TeamMessage).name();
  TypeRegistry::addClass(name, nullptr);
#define REGISTER_TEAM_MESSAGE_REPRESENTATION(x) TypeRegistry::addAttribute(name, typeid(x).name(), "the" #x)

  REGISTER_TEAM_MESSAGE_REPRESENTATION(RobotStatus);
  REGISTER_TEAM_MESSAGE_REPRESENTATION(RobotPose);
  FOREACH_TEAM_MESSAGE_REPRESENTATION(REGISTER_TEAM_MESSAGE_REPRESENTATION);
}

TeamMessageHandler::TeamMessageHandler() :
  theBNTP(theFrameInfo, theRobotInfo)
{
  File f("teamMessage.def", "r");
  ASSERT(f.exists());
  std::string source(f.getSize(), 0);
  f.read(source.data(), source.length());
  teamCommunicationTypeRegistry.addTypes(source);
  teamCommunicationTypeRegistry.compile();
  teamMessageType = teamCommunicationTypeRegistry.getTypeByName("TeamMessage");
}

void TeamMessageHandler::update(NaovaMessageOutputGenerator& outputGenerator)
{
  DEBUG_RESPONSE_ONCE("module:TeamMessageHandler:generateTCMPluginClass")
    teamCommunicationTypeRegistry.generateTCMPluginClass("BHumanStandardMessage.java", static_cast<const CompressedTeamCommunication::RecordType*>(teamMessageType));

  outputGenerator.sendThisFrame =
#ifndef SITTING_TEST
#ifdef TARGET_ROBOT
    theMotionRequest.motion != MotionRequest::playDead &&
    theMotionInfo.executedPhase != MotionPhase::playDead &&
#endif
#endif // !SITTING_TEST
    sendMessage();

  theRobotStatus.hasGroundContact = theGroundContactState.contact && theMotionInfo.executedPhase != MotionPhase::getUp && theMotionRequest.motion != MotionRequest::getUp;
  theRobotStatus.isUpright = theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting;
  if(theRobotStatus.hasGroundContact)
    theRobotStatus.timeOfLastGroundContact = theFrameInfo.time;
  if(theRobotStatus.isUpright)
    theRobotStatus.timeWhenLastUpright = theFrameInfo.time;

  outputGenerator.generate = [this, &outputGenerator](RoboCup::SPLStandardMessage* const m)
  {
    generateMessage(outputGenerator);
    writeMessage(outputGenerator, m);

    // snap current data before sent
    snap.lastPose = theRobotPose.translation;
    snap.lastTimeSendMessage = Time::getCurrentSystemTime();
    snap.lastPenaltyState = theRobotInfo.penalty;
    snap.lastBallPosition = theBallModel.estimate.position;
  };
}

void TeamMessageHandler::generateMessage(NaovaMessageOutputGenerator& outputGenerator) const
{
#define SEND_PARTICLE(particle) \
  the##particle >> outputGenerator

  outputGenerator.theNaovaStandardMessage.number = static_cast<uint8_t>(theRobotInfo.number);
  outputGenerator.theNaovaStandardMessage.magicNumber = Global::getSettings().magicNumber;

  outputGenerator.theNaovaStandardMessage.timestamp = Time::getCurrentSystemTime();

  theRobotStatus.isPenalized = theRobotInfo.penalty != PENALTY_NONE;

  outputGenerator.theNaovaStandardMessage.compressedContainer.reserve(SPL_STANDARD_MESSAGE_DATA_SIZE);
  CompressedTeamCommunicationOut stream(outputGenerator.theNaovaStandardMessage.compressedContainer, outputGenerator.theNaovaStandardMessage.timestamp,
                                        teamMessageType, !outputGenerator.sentMessages);
  outputGenerator.theNaovaStandardMessage.out = &stream;

  SEND_PARTICLE(RobotStatus);

  if(sendMirroredRobotPose)
  {
    RobotPose theMirroredRobotPose = theRobotPose;
    theMirroredRobotPose.translation *= -1.f;
    theMirroredRobotPose.rotation = Angle::normalize(theMirroredRobotPose.rotation + pi);
    SEND_PARTICLE(MirroredRobotPose);
  }
  else
    SEND_PARTICLE(RobotPose);

  FOREACH_TEAM_MESSAGE_REPRESENTATION(SEND_PARTICLE);

  outputGenerator.theNaovaStandardMessage.out = nullptr;
}

void TeamMessageHandler::writeMessage(NaovaMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const
{
  ASSERT(outputGenerator.sendThisFrame);

  outputGenerator.theNaovaStandardMessage.write(reinterpret_cast<void*>(m->data));

  ASSERT(outputGenerator.theNaovaStandardMessage.sizeOfNaovaMessage() <= SPL_STANDARD_MESSAGE_DATA_SIZE);

  outputGenerator.sentMessages++;
  if(theFrameInfo.getTimeSince(timeLastSent) >= 2 * sendInterval)
    timeLastSent = theFrameInfo.time;
  else
    timeLastSent += sendInterval;
}

void TeamMessageHandler::update(TeamData& teamData)
{
  teamData.generate = [this, &teamData](const SPLStandardMessageBufferEntry* const m)
  {
    if(readSPLStandardMessage(m))
    {
      if (receivedMessageContainer.theNaovaStandardMessage.number != theRobotInfo.number)
        parseMessageIntoBMate(getBMate(teamData));
      return;
    }

    if(receivedMessageContainer.lastErrorCode == ReceivedNaovaMessage::myOwnMessage
#ifndef NDEBUG
       || receivedMessageContainer.lastErrorCode == ReceivedNaovaMessage::magicNumberDidNotMatch
#endif
      ) return;

    //the message had an parsing error
    if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruderAlert.wav"))
      timeWhenLastMimimi = theFrameInfo.time;

    ANNOTATION("intruder-alert", "error code: " << receivedMessageContainer.lastErrorCode);
  };

  maintainBMateList(teamData);
}

void TeamMessageHandler::maintainBMateList(TeamData& teamData) const
{
  //@author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
  {
    // Iterate over deprecated list of teammate information and update some convenience information
    // (new information has already been coming via handleMessages)
    for(auto& teammate : teamData.teammates)
    {
      Teammate::Status newStatus = Teammate::PLAYING;
      if(teammate.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
        newStatus = Teammate::PENALIZED;
      else if(!teammate.isUpright || !teammate.hasGroundContact)
        newStatus = Teammate::FALLEN;

      if(newStatus != teammate.status)
      {
        teammate.status = newStatus;
        teammate.timeWhenStatusChanged = theFrameInfo.time;
      }

      teammate.isGoalkeeper = teammate.number == 1;
    }
    // Remove elements that are too old:
    auto teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(teammate->isPenalized)
        teammate = teamData.teammates.erase(teammate);
      else
        ++teammate;
    }

    // Other stuff
    teamData.numberOfActiveTeammates = 0;
    teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(teammate->status != Teammate::PENALIZED)
        teamData.numberOfActiveTeammates++;
      ++teammate;
    }
  }
}

#define PARSING_ERROR(outputText) { OUTPUT_ERROR(outputText); receivedMessageContainer.lastErrorCode = ReceivedNaovaMessage::parsingError;  return false; }
bool TeamMessageHandler::readSPLStandardMessage(const SPLStandardMessageBufferEntry* const m)
{
  if(!receivedMessageContainer.theNaovaStandardMessage.read(m->message.data))
    PARSING_ERROR(NAOVA_STANDARD_MESSAGE_STRUCT_HEADER " message part reading failed");

  receivedMessageContainer.timestamp = m->timestamp;

  return true;
}

Teammate& TeamMessageHandler::getBMate(TeamData& teamData) const
{
  teamData.receivedMessages++;

  for(auto& teammate : teamData.teammates)
    if(teammate.number == receivedMessageContainer.theNaovaStandardMessage.number)
      return teammate;

  teamData.teammates.emplace_back();
  return teamData.teammates.back();
}

#define RECEIVE_PARTICLE(particle) currentTeammate.the##particle << receivedMessageContainer
void TeamMessageHandler::parseMessageIntoBMate(Teammate& currentTeammate)
{
  currentTeammate.number = receivedMessageContainer.theNaovaStandardMessage.number;

  receivedMessageContainer.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.timeWhenLastPacketSent = receivedMessageContainer.toLocalTimestamp(receivedMessageContainer.theNaovaStandardMessage.timestamp);
  currentTeammate.timeWhenLastPacketReceived = receivedMessageContainer.timestamp;

  CompressedTeamCommunicationIn stream(receivedMessageContainer.theNaovaStandardMessage.compressedContainer,
                                       receivedMessageContainer.theNaovaStandardMessage.timestamp, teamMessageType,
                                       [this](unsigned u) { return receivedMessageContainer.toLocalTimestamp(u); });
  receivedMessageContainer.theNaovaStandardMessage.in = &stream;

  RobotStatus robotStatus;
  robotStatus << receivedMessageContainer;

  currentTeammate.isPenalized = robotStatus.isPenalized;
  currentTeammate.isUpright = robotStatus.isUpright;
  currentTeammate.hasGroundContact = robotStatus.hasGroundContact;
  currentTeammate.timeWhenLastUpright = robotStatus.timeWhenLastUpright;
  currentTeammate.timeOfLastGroundContact = robotStatus.timeOfLastGroundContact;

  RECEIVE_PARTICLE(RobotPose);
  FOREACH_TEAM_MESSAGE_REPRESENTATION(RECEIVE_PARTICLE);

  receivedMessageContainer.theNaovaStandardMessage.in = nullptr;

}

//@author #iloveCatarina
bool TeamMessageHandler::sendMessage(){
  if (theFrameInfo.getTimeSince(timeLastSent) >= sendInterval && enableCommunication1s)
    return true;

  if (getRemainingMessages() <= securityMessage) {
    return false;
  }

  bool send = false;
  uint8_t stateGameInfo;

  #ifdef TARGET_ROBOT
    stateGameInfo = theExtendedGameInfo.gameStateLastFrame;
  #else
    if ((theRawGameInfo.state == STATE_READY || theRawGameInfo.state == STATE_PLAYING) && snap.firstMessage) { 
      snap.firstMessage = false;
      return true;
    }  
    stateGameInfo = theRawGameInfo.state;
  #endif
  
  switch (stateGameInfo)
    {
    case STATE_INITIAL:
      snap.firstMessage = true;
      break;
    case STATE_READY:
      send = theExtendedGameInfo.gameStateBeforeCurrent != STATE_INITIAL ? getScoreRobotMoved() >= minScore : false;
      break;
    case STATE_SET: // Send message when a whistle is heard
      if (getScoreWhistleSET() >= minScore && (theFrameInfo.getTimeSince(snap.lastTimeSendMessage) >= sendInterval)) {
        send = true;
      } 
      break;
    default: // STATE_PLAYING
      adjustScore();

      int scoreRobotMoved = getScoreRobotMoved();
      int scoreBallMoved = getScoreBallMoved();
      int scoreBallDetected = getScoreBallDetected();
      int scoreFallen = getScoreFallen();
      int scoreWhistle = getScoreWhistle();
      int scorePenalized = getScorePenalized();
      int scoreUnpenalized = getScoreUnpenalized();
      

      int scoreTotal = scoreFallen + scoreBallMoved + scoreBallDetected + scorePenalized + scoreUnpenalized + scoreRobotMoved;
      send = scoreTotal >= minScore;
      break;
  }
  
  return send;
}

void TeamMessageHandler::log(std::map<std::string, std::string> values, std::string reasonForLogging) {
  OUTPUT_TEXT("\n\n\n(" << theRobotInfo.number << "): " << reasonForLogging << " ---------------------------------------------------------------");
  OUTPUT_TEXT("(" << theRobotInfo.number << "): currentSystemTime: " << Time::getCurrentSystemTime() << "\n");
  for(auto v : values) {
    OUTPUT_TEXT("(" << theRobotInfo.number << "): " << v.first << ": " << v.second << "\n");
  }
}

void TeamMessageHandler::log(std::string value, std::string reasonForLogging) {
  OUTPUT_TEXT("\n\n\n(" << theRobotInfo.number << "): " << reasonForLogging << " ---------------------------------------------------------------");
  OUTPUT_TEXT("(" << theRobotInfo.number << "): " << value);
}

void TeamMessageHandler::adjustScore(){
  int secsInHalf = totalSecsGame/2;
  int remainingSeconds = theRawGameInfo.firstHalf == 1 ? secsInHalf + theRawGameInfo.secsRemaining : theRawGameInfo.secsRemaining;
  int elapsedSeconds = (secsInHalf*2)-remainingSeconds;

  float scoreSlopeDeltaX = (float)((limitMessages*2)/8);
  float scoreSlope = (float)(scoreFloor-scoreCeiling)/scoreSlopeDeltaX;
  float targetSentPacketsSlope = (float)((limitMessages)/(secsInHalf*2));

  int expectedNumberOfSentPackets = (int)(targetSentPacketsSlope*elapsedSeconds);
  int actualNumberOfSentPackets = (limitMessages - getRemainingMessages());

  int minScoreTemp = (int)(scoreSlope*(expectedNumberOfSentPackets-actualNumberOfSentPackets) + defaultMinScore);

  if (minScoreTemp < scoreFloor) {
    minScore = scoreFloor;
  } else if (minScoreTemp > scoreCeiling) {
    minScore = scoreCeiling;
  } else {
    minScore = minScoreTemp;
  }
}


int TeamMessageHandler::getScoreBallMoved(){

  Vector2f currentBallPosition = theBallModel.estimate.position;
  Vector2f lastBallPosition = snap.lastBallPosition;
  float difference = (currentBallPosition - lastBallPosition).norm();

  if (difference < minBallRadius || theBallPercept.status != BallPercept::seen ){
    return 0;
  }

  return (ballMoved * (int)(difference/(maxBallRadius - minBallRadius)));
}

int TeamMessageHandler::getScoreBallDetected() {
  if (theBallPercept.status == BallPercept::seen && theFrameInfo.getTimeSince(snap.lastTimeBallDetectedMessage) > minBallDetectedTime) {
    snap.lastTimeBallDetectedMessage = theFrameInfo.time;
    return minScore;
  }
  return 0;
}

int TeamMessageHandler::getScoreRobotMoved(){
  Vector2f lastPosition = snap.lastPose;
  Vector2f currentPosition = theRobotPose.translation;

  float difference = ((currentPosition - lastPosition)).norm();

  if (difference < minDistanceRobot) {
    return 0;
  }

  return (robotMoved * (int)(difference/(maxDistanceRobot-minDistanceRobot)));
}

int TeamMessageHandler::getScoreFallen(){
  if (theFallDownState.state != FallDownState::State::fallen && theFallDownState.state != FallDownState::State::falling){
    return 0;
  }

  float distanceRobotBall = theBallModel.estimate.position.norm();

  if (distanceRobotBall > maxFallenDistance){
    return 0;
  }

  int fallenScore = (int)(fallen * (1.0 - (distanceRobotBall/maxFallenDistance)));
  return fallenScore;
}

int TeamMessageHandler::getScoreWhistle(){
  float difference = (float)theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected);
  if (difference <= maxWhistleTime && theWhistle.confidenceOfLastWhistleDetection >= minWhistleConfidence) {
    return (int)(whistle * (1.0f - (difference/maxWhistleTime)));
  }
  return 0;
}

int TeamMessageHandler::getScoreWhistleSET() {
  if (theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) <= maxWhistleTime && theWhistle.confidenceOfLastWhistleDetection >= minWhistleConfidence) {
    return minScore;
  }
  return 0;
}

int TeamMessageHandler::getScorePenalized(){
  return theRobotInfo.penalty != PENALTY_NONE && snap.lastPenaltyState == PENALTY_NONE ? minScore : 0;
}

int TeamMessageHandler::getScoreUnpenalized(){
  return theRobotInfo.penalty == PENALTY_NONE && snap.lastPenaltyState != PENALTY_NONE ? minScore : 0;
}

int TeamMessageHandler::getRemainingMessages(){
  #ifdef TARGET_ROBOT
    return theOwnTeamInfo.messageBudget;
  #endif
  return limitMessages - (nbReceivedMessages + nbSentMessages);
}