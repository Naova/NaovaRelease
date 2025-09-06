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

struct TeamMessage
{};

#define REGISTER_TEAM_MESSAGE_REPRESENTATION(x) TypeRegistry::addAttribute(name, typeid(x).name(), "the" #x)
void TeamMessageHandler::regTeamMessage()
{
  PUBLISH(regTeamMessage);
  const char* name = typeid(TeamMessage).name();
  TypeRegistry::addClass(name, nullptr);
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
  messageCount = 0;
}

void TeamMessageHandler::changeMessageType(){
  std::string usedRepresentations = "";
  for (int i = 0; i < NUMBER_OF_REPRESENTATIONS; i++){
    if (getBitValue(currentMessageType, i) == 1){
        usedRepresentations += representationOptions[i];
    }
  }
  teamCommunicationTypeRegistry.addTypes("\nObstacleModel\n{\nobstacles: Obstacle[:" + std::to_string(maxObstacles) + "]\n}\n");
  teamCommunicationTypeRegistry.addTypes("\nTeamMessage\n{\n"+ usedRepresentations + "}\n");
  teamCommunicationTypeRegistry.compile();
  teamMessageType = teamCommunicationTypeRegistry.getTypeByName("TeamMessage"); 
}

bool TeamMessageHandler::getBitValue(uint16_t compressed_header, int index){
  return (compressed_header >> index) & 1;
}

void TeamMessageHandler::setBitValue(uint16_t& compressed_header, int index, bool value){
  if (value){
    compressed_header |= 1 << index;
  } else {
    compressed_header &= ~(1 << index);
  }
}

void TeamMessageHandler::addToMessage(int& messageSize, int representationIndex){
  if (representationSizes[representationIndex] + messageSize <= SPL_STANDARD_MESSAGE_DATA_SIZE){
    setBitValue(currentMessageType, representationIndex, true);
    messageSize += representationSizes[representationIndex] ;
  }
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
  int messageSize = EMPTY_MESSAGE_SIZE;
  currentMessageType = 0;
  maxObstacles = 1;
  int counter = 0;

  while (messageSize < SPL_STANDARD_MESSAGE_DATA_SIZE && counter < NUMBER_OF_REPRESENTATIONS)
  {
    if (counter == ballModelIndex){
      if (getScoreBallMoved() > 0 || (lastFrameState == STATE_SET && theGameInfo.state == STATE_PLAYING)){
        addToMessage(messageSize, counter);
      }
    }
    else if (counter == robotPoseIndex){
      if (((theRobotPose.translation - snap.lastPose)).norm() > minDistanceToSendPose)
        addToMessage(messageSize, counter);
    }
    else
      addToMessage(messageSize, counter);
    counter++;
  }
  //Pour nombre d'obstacles variables
  if (messageSize <= SPL_STANDARD_MESSAGE_DATA_SIZE)
  {
    maxObstacles += (int) floor((SPL_STANDARD_MESSAGE_DATA_SIZE - messageSize) / representationSizes[obstacleModelIndex]);
    messageSize += (maxObstacles-1) * representationSizes[obstacleModelIndex];
  }

  lastFrameState = theGameInfo.state;
  outputGenerator.generate = [this, &outputGenerator](RoboCup::SPLStandardMessage* const m)
  {
    generateMessage(outputGenerator);
    if (outputGenerator.theNaovaStandardMessage.sizeOfNaovaMessage() > SPL_STANDARD_MESSAGE_DATA_SIZE)
      return;
    writeMessage(outputGenerator, m);

    // snap current data before sent
    snap.lastPose = theRobotPose.translation;
    snap.lastTimeSendMessage = Time::getCurrentSystemTime();
    snap.lastPenaltyState = theRobotInfo.penalty;
    snap.lastBallPosition = theBallModel.estimate.position;
  };
}

#define SEND_PARTICLE(particle) the##particle >> outputGenerator
void TeamMessageHandler::generateMessage(NaovaMessageOutputGenerator& outputGenerator)
{
  outputGenerator.theNaovaStandardMessage.number = static_cast<uint8_t>(theRobotInfo.number);
  outputGenerator.theNaovaStandardMessage.magicNumber = Global::getSettings().magicNumber;

  outputGenerator.theNaovaStandardMessage.timestamp = Time::getCurrentSystemTime();

  outputGenerator.theNaovaStandardMessage.header = currentMessageType;
  changeMessageType();
  theRobotStatus.isPenalized = theRobotInfo.penalty != PENALTY_NONE;
  
  outputGenerator.theNaovaStandardMessage.compressedContainer.reserve(SPL_STANDARD_MESSAGE_DATA_SIZE);
  CompressedTeamCommunicationOut stream(outputGenerator.theNaovaStandardMessage.compressedContainer, outputGenerator.theNaovaStandardMessage.timestamp,
                                        teamMessageType, !outputGenerator.sentMessages);
  outputGenerator.theNaovaStandardMessage.out = &stream;

  //Garde en cas de besoins futurs
  // if(sendMirroredRobotPose)
  // {
  //   RobotPose theMirroredRobotPose = theRobotPose;
  //   theMirroredRobotPose.translation *= -1.f;
  //   theMirroredRobotPose.rotation = Angle::normalize(theMirroredRobotPose.rotation + pi);
  // }  //   SEND_PARTICLE(MirroredRobotPose);

  if (getBitValue(currentMessageType, robotStatusIndex) == true)
    SEND_PARTICLE(RobotStatus);
  if (getBitValue(currentMessageType, robotPoseIndex) == true)
    SEND_PARTICLE(RobotPose);
  if (getBitValue(currentMessageType, refereeReadySignalIndex) == true)
    SEND_PARTICLE(RefereeReadySignal);
  if (getBitValue(currentMessageType, ballModelIndex) == true)
    SEND_PARTICLE(BallModel);
  if (getBitValue(currentMessageType, obstacleModelIndex) == true)
    SEND_PARTICLE(ObstacleModel);
  if (getBitValue(currentMessageType, whistleIndex) == true)
    SEND_PARTICLE(Whistle);
  if (getBitValue(currentMessageType, behaviorStatusIndex) == true)
    SEND_PARTICLE(BehaviorStatus);
  if (getBitValue(currentMessageType, teamBehaviorStatusIndex) == true)
    SEND_PARTICLE(TeamBehaviorStatus);
  if (getBitValue(currentMessageType, robotHadBallContactIndex) == true)
    SEND_PARTICLE(RobotHadBallContact);
  
  messageCount++;
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
      if(teammate.theRobotStatus.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
        newStatus = Teammate::PENALIZED;
      else if(!teammate.theRobotStatus.isUpright || !teammate.theRobotStatus.hasGroundContact)
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
      if(teammate->theRobotStatus.isPenalized)
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

#define RECEIVE_PARTICLE(particle) currentTeammate.the##particle << receivedMessageContainer;
void TeamMessageHandler::parseMessageIntoBMate(Teammate& currentTeammate)
{
  currentTeammate.number = receivedMessageContainer.theNaovaStandardMessage.number;

  receivedMessageContainer.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.timeWhenLastPacketSent = receivedMessageContainer.toLocalTimestamp(receivedMessageContainer.theNaovaStandardMessage.timestamp);
  currentTeammate.timeWhenLastPacketReceived = receivedMessageContainer.timestamp;

  currentMessageType = receivedMessageContainer.theNaovaStandardMessage.header;
  changeMessageType();
  CompressedTeamCommunicationIn stream(receivedMessageContainer.theNaovaStandardMessage.compressedContainer,
                                       receivedMessageContainer.theNaovaStandardMessage.timestamp, teamMessageType,
                                       [this](unsigned u) { return receivedMessageContainer.toLocalTimestamp(u); });
  receivedMessageContainer.theNaovaStandardMessage.in = &stream;

  if (getBitValue(currentMessageType, robotStatusIndex) == true)
    RECEIVE_PARTICLE(RobotStatus);
  if (getBitValue(currentMessageType, robotPoseIndex) == true)
    RECEIVE_PARTICLE(RobotPose);
  if (getBitValue(currentMessageType, refereeReadySignalIndex) == true)
    RECEIVE_PARTICLE(RefereeReadySignal);
  if (getBitValue(currentMessageType, ballModelIndex) == true)
    RECEIVE_PARTICLE(BallModel);
  if (getBitValue(currentMessageType, obstacleModelIndex) == true)
    RECEIVE_PARTICLE(ObstacleModel);
  if (getBitValue(currentMessageType, whistleIndex) == true)
    RECEIVE_PARTICLE(Whistle);
  if (getBitValue(currentMessageType, behaviorStatusIndex) == true)
    RECEIVE_PARTICLE(BehaviorStatus);
  if (getBitValue(currentMessageType, teamBehaviorStatusIndex) == true)
    RECEIVE_PARTICLE(TeamBehaviorStatus);
  if (getBitValue(currentMessageType, robotHadBallContactIndex) == true)
    RECEIVE_PARTICLE(RobotHadBallContact);

  messageCount++;
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
      send = !IS_PREGAME_STATE(theExtendedGameInfo.gameStateBeforeCurrent) ? getScoreRobotMoved() >= minScore : false;
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
      int scoreRobotBallContact = getScoreRobotBallContact();
      int scoreRefereeDetection = getScoreRefereeReadySignal();
      int scoreKickoff = getScoreKickoff();

      int scoreTotal = scoreFallen + scoreBallMoved + scoreBallDetected + scorePenalized + scoreUnpenalized + scoreRobotMoved + scoreRobotBallContact + scoreRefereeDetection + scoreKickoff;
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

int TeamMessageHandler::getScoreRobotBallContact() {
  // if (theRobotHadBallContact.hadContact && theTeamData.someoneCantShoot()) {
  //   return minScore;
  // }
  return 0;
}

int TeamMessageHandler::getScoreKickoff() {
  if (lastFrameState == STATE_SET && theGameInfo.state == STATE_PLAYING) {
    return minScore;
  }
  return 0;
}

int TeamMessageHandler::getScoreRefereeReadySignal() {
  bool inRelevantState = theRawGameInfo.state == STATE_STANDBY || theRawGameInfo.state == STATE_READY;
  if (inRelevantState && theRefereeReadySignal.isDetected && !refereeDetectedLastState) {
    refereeDetectedLastState = theRefereeReadySignal.isDetected;
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