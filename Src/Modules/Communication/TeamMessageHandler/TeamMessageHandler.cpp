/**
 * @file TeamMessageHandler.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "TeamMessageHandler.h"
#include "Tools/MessageQueue/OutMessage.h"
#include "Tools/Global.h"
#include "Platform/Time.h"

//#define SITTING_TEST
//#define SELF_TEST

/**
 * A macro for broadcasting team messages.
 * @param type The type of the message from the MessageID enum in MessageIDs.h
 * @param format The message format of the message (bin or text).
 * @param expression A streamable expression.
 */
#define TEAM_OUTPUT(type,format,expression) \
  { Global::getTeamOut().format << expression;\
    Global::getTeamOut().finishMessage(type); }

MAKE_MODULE(TeamMessageHandler, communication)

void TeamMessageHandler::update(NaovaMessageOutputGenerator& outputGenerator)
{
  outputGenerator.theBHumanArbitraryMessage.queue.clear();

  DEBUG_RESPONSE_ONCE("module:TeamMessageHandler:toggleLogging") {
    loggingIsEnabled = !loggingIsEnabled;
  }

  outputGenerator.sendThisFrame =
#ifndef SITTING_TEST
#ifdef TARGET_ROBOT
    !(theMotionRequest.motion == MotionRequest::specialAction && theMotionRequest.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
    !(theMotionInfo.motion == MotionRequest::specialAction && theMotionInfo.specialActionRequest.specialAction == SpecialActionRequest::playDead) &&
#endif
#endif // !SITTING_TEST
    sendMessage();

  outputGenerator.generate = [this, &outputGenerator](RoboCup::SPLStandardMessage* const m)
  {
    generateMessage(outputGenerator);
    writeMessage(outputGenerator, m);
    // snap current data before sent
    snap.lastPose = theRobotPose.translation;
    snap.lastTimeSendMessage = Time::getCurrentSystemTime();
    snap.lastPenaltyState = theRobotInfo.penalty;
    snap.lastMessageRole = theBehaviorStatus.role;
    snap.lastBallPosition = theBallModel.estimate.position;
    snap.lastGameState = theGameInfo.state;
  };
}

void TeamMessageHandler::generateMessage(NaovaMessageOutputGenerator& outputGenerator) const
{
#define SEND_PARTICLE(particle) \
  the##particle >> outputGenerator

  outputGenerator.theNaovaSPLStandardMessage.playerNum = static_cast<uint8_t>(theRobotInfo.number);
  outputGenerator.theNaovaSPLStandardMessage.teamNum = static_cast<uint8_t>(Global::getSettings().teamNumber);
  outputGenerator.theNaovaStandardMessage.magicNumber = Global::getSettings().magicNumber;

  outputGenerator.theNaovaStandardMessage.timestamp = Time::getCurrentSystemTime();

  outputGenerator.theNaovaStandardMessage.hasGroundContact = theGroundContactState.contact && theMotionInfo.motion != MotionInfo::getUp && theMotionRequest.motion != MotionRequest::getUp;
  outputGenerator.theNaovaStandardMessage.isUpright = theFallDownState.state == theFallDownState.upright;
  outputGenerator.theNaovaStandardMessage.isPenalized = theRobotInfo.penalty != PENALTY_NONE;

  outputGenerator.theNaovaSPLStandardMessage.fallen = !outputGenerator.theNaovaStandardMessage.hasGroundContact || ! outputGenerator.theNaovaStandardMessage.isUpright;

  // SEND_PARTICLE(BNTP);
  SEND_PARTICLE(RawGameInfo);
  // SEND_PARTICLE(OwnTeamInfo);

  SEND_PARTICLE(BallModel);
  SEND_PARTICLE(RobotPose);

  // SEND_PARTICLE(SideConfidence);
  // SEND_PARTICLE(TeammateRoles);
  SEND_PARTICLE(BehaviorStatus);
  // SEND_PARTICLE(SPLStandardBehaviorStatus);

  SEND_PARTICLE(Whistle);

  //Send this last of important data, because they are the biggest
  // SEND_PARTICLE(ObstacleModel);
  SEND_PARTICLE(FieldCoverage);

  //Send this last, because it is unimportant for robots, (so it is ok, if it gets dropped)
  // SEND_PARTICLE(RobotHealth);
  // SEND_PARTICLE(FieldFeatureOverview);

  outputGenerator.theNaovaSPLStandardMessage.numOfDataBytes =
    static_cast<uint16_t>(outputGenerator.theNaovaStandardMessage.sizeOfNaovaMessage()
                          + outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage());
}

void TeamMessageHandler::writeMessage(NaovaMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const
{
  ASSERT(outputGenerator.sendThisFrame);

  outputGenerator.theNaovaStandardMessage.write(reinterpret_cast<void*>(m->data));
  int offset = outputGenerator.theNaovaStandardMessage.sizeOfNaovaMessage();

  const int restBytes = SPL_STANDARD_MESSAGE_DATA_SIZE - offset;
  ASSERT(restBytes > 10);

  int sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage();
  // fprintf(stderr, "Before NumofByte: %d\n", sizeOfArbitraryMessage); // TODO Remove avant merge
  if(sizeOfArbitraryMessage > restBytes + 1)
  {
    // OUTPUT_ERROR("outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage() > restBytes "
                //  "-- with size of " << sizeOfArbitraryMessage << " and restBytes:" << int(restBytes));

    do
      outputGenerator.theBHumanArbitraryMessage.queue.removeLastMessage();
    while((sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage()) > restBytes
          && !outputGenerator.theBHumanArbitraryMessage.queue.isEmpty());
  }
  sizeOfArbitraryMessage = outputGenerator.theBHumanArbitraryMessage.sizeOfArbitraryMessage();
  ASSERT(sizeOfArbitraryMessage <= restBytes);

  outputGenerator.theBHumanArbitraryMessage.write(reinterpret_cast<void*>(m->data + offset));

  outputGenerator.theNaovaSPLStandardMessage.numOfDataBytes = static_cast<uint16_t>(offset + sizeOfArbitraryMessage);
  // fprintf(stderr, "After NumofByte: %d\n", outputGenerator.theNaovaSPLStandardMessage.numOfDataBytes); // TODO avant merge
  outputGenerator.theNaovaSPLStandardMessage.write(reinterpret_cast<void*>(&m->header[0]));

  outputGenerator.sentMessages++;
  timeLastSent = theFrameInfo.time;
  nbSentMessages = outputGenerator.sentMessages;
}

void TeamMessageHandler::update(TeamData& teamData)
{
  teamData.generate = [this, &teamData](const RoboCup::SPLStandardMessage* const m)
  {
    if(readSPLStandardMessage(m))
      return parseMessageIntoBMate(getBMate(teamData));

    if(receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::myOwnMessage
#ifndef NDEBUG
       || receivedMessageContainer.lastErrorCode == ReceivedBHumanMessage::magicNumberDidNotMatch
#endif
      ) return;

    //the message had an parsing error
    if(theFrameInfo.getTimeSince(timeWhenLastMimimi) > minTimeBetween2RejectSounds && SystemCall::playSound("intruder-alert.wav"))
      timeWhenLastMimimi = theFrameInfo.time;

    ANNOTATION("intruder-alert", "error code: " << receivedMessageContainer.lastErrorCode);
  };

  teamData.canSendMessages = (limitMessages - getRemainingMessages()) < (limitMessages - securityMessage);
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
      if(teammate.isPenalized || theOwnTeamInfo.players[teammate.number - 1].penalty != PENALTY_NONE)
        teammate.status = Teammate::PENALIZED;
      else if(!teammate.isUpright || !teammate.hasGroundContact)
        teammate.status = Teammate::FALLEN;
      else
        teammate.status = Teammate::PLAYING;

      teammate.isGoalkeeper = teammate.number == 1;
    }

    // Remove elements that are too old:
    auto teammate = teamData.teammates.begin();
    while(teammate != teamData.teammates.end())
    {
      if(teammate->isPenalized) // Condition pour enlever un teammate de la liste
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

#define PARSING_ERROR(outputText) { OUTPUT_ERROR(outputText); receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::parsingError;  return false; }
bool TeamMessageHandler::readSPLStandardMessage(const RoboCup::SPLStandardMessage* const m)
{
  if(!receivedMessageContainer.theNaovaSPLStandardMessage.read(&m->header[0]))
    PARSING_ERROR("BSPL" " message part reading failed");

#ifndef SELF_TEST
  if(receivedMessageContainer.theNaovaSPLStandardMessage.playerNum == theRobotInfo.number)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::myOwnMessage) && false;
#endif // !SELF_TEST

  if(receivedMessageContainer.theNaovaSPLStandardMessage.playerNum < Settings::lowestValidPlayerNumber ||
     receivedMessageContainer.theNaovaSPLStandardMessage.playerNum > Settings::highestValidPlayerNumber)
    PARSING_ERROR("Invalid robot number received");

  if(receivedMessageContainer.theNaovaSPLStandardMessage.teamNum != static_cast<uint8_t>(Global::getSettings().teamNumber))
    PARSING_ERROR("Invalid team number received");

  if(!receivedMessageContainer.theNaovaStandardMessage.read(m->data))
    PARSING_ERROR(NAOVA_STANDARD_MESSAGE_STRUCT_HEADER " message part reading failed");

  if(receivedMessageContainer.theNaovaStandardMessage.magicNumber != Global::getSettings().magicNumber)
    return (receivedMessageContainer.lastErrorCode = ReceivedBHumanMessage::magicNumberDidNotMatch) && false;

  return true;
}

Teammate& TeamMessageHandler::getBMate(TeamData& teamData) const
{
  teamData.receivedMessages++;
  nbReceivedMessages = teamData.receivedMessages;

  for(auto& teammate : teamData.teammates)
    if(teammate.number == receivedMessageContainer.theNaovaSPLStandardMessage.playerNum)
      return teammate;

  teamData.teammates.emplace_back();
  return teamData.teammates.back();
}

#define RECEIVE_PARTICLE(particle) currentTeammate.the##particle << receivedMessageContainer
void TeamMessageHandler::parseMessageIntoBMate(Teammate& currentTeammate)
{
  // fprintf(stderr,
  //    "-----------------------------------------------------------------------------\n\nNaovaSPLStandardMessage : \n version : %d\n playerNum : %d\n teamNum : %d\n fallen : %d\n pose : %f %f %f\n ballAge : %f\n ball : %f %f\n",
  //    receivedMessageContainer.theNaovaSPLStandardMessage.version,
  //    receivedMessageContainer.theNaovaSPLStandardMessage.playerNum,
  //    receivedMessageContainer.theNaovaSPLStandardMessage.teamNum,
  //    receivedMessageContainer.theNaovaSPLStandardMessage.fallen,
  //    receivedMessageContainer.theNaovaSPLStandardMessage.pose[0],receivedMessageContainer.theNaovaSPLStandardMessage.pose[1], receivedMessageContainer.theNaovaSPLStandardMessage.pose[2],
  //    receivedMessageContainer.theNaovaSPLStandardMessage.ballAge,
  //    receivedMessageContainer.theNaovaSPLStandardMessage.ball[0], receivedMessageContainer.theNaovaSPLStandardMessage.ball[1]);
  // fprintf(stderr,
  //    "NaovaStandardMessage : \n magicNumber : %d\n ballLastPerceptX : %d\n ballLastPerceptY : %d\n ballLastPerceptY : %d\n ballCovariance : %f %f %f\n robotPoseDeviation: %f\n",
  //   //  receivedMessageContainer.theNaovaStandardMessage.version,
  //    receivedMessageContainer.theNaovaStandardMessage.magicNumber,
  //   //  receivedMessageContainer.theNaovaStandardMessage.ballTimeWhenDisappearedSeenPercentage,
  //    receivedMessageContainer.theNaovaStandardMessage.ballLastPerceptX,
  //    receivedMessageContainer.theNaovaStandardMessage.ballLastPerceptY,
  //    receivedMessageContainer.theNaovaStandardMessage.ballCovariance[0], receivedMessageContainer.theNaovaStandardMessage.ballCovariance[1], receivedMessageContainer.theNaovaStandardMessage.ballCovariance[2],
  //    receivedMessageContainer.theNaovaStandardMessage.robotPoseDeviation);
  // fprintf(stderr,
  //    " robotPoseValidity : %d\n isPenalized : %s\n isUpright : %s\n hasGroundContact : %s\n timestamp : %d\n",
  //   //  outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[0], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[1], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[2], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[3], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[4], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[5],
  //    receivedMessageContainer.theNaovaStandardMessage.robotPoseValidity,
  //    receivedMessageContainer.theNaovaStandardMessage.isPenalized ? "true" : "false",
  //    receivedMessageContainer.theNaovaStandardMessage.isUpright ? "true" : "false",
  //    receivedMessageContainer.theNaovaStandardMessage.hasGroundContact ? "true" : "false",
  //    receivedMessageContainer.theNaovaStandardMessage.timestamp);
  //   //  outTeamMessage.theNaovaStandardMessage.keeperIsPlaying ? "true" : "false",
  //   //  outTeamMessage.theNaovaStandardMessage.passTarget,
  //   //  receivedMessageContainer.theNaovaStandardMessage.timeWhenReachBall,
  //   //  receivedMessageContainer.theNaovaStandardMessage.timeWhenReachBallStriker);
  //    fprintf(stderr,
  //    " timestampLastJumped :  %d\n ballTimeWhenLastSeen : %d\n lastTimeWhistleDetected : %d\n--------------------------------------------------\n\n",
  //    receivedMessageContainer.theNaovaStandardMessage.timestampLastJumped,
  //    receivedMessageContainer.theNaovaStandardMessage.ballTimeWhenLastSeen,
  //    receivedMessageContainer.theNaovaStandardMessage.lastTimeWhistleDetected
  //   //  outTeamMessage.theNaovaStandardMessage.requestsNTPMessage ? "true" : "false"
  //   );
    
  currentTeammate.number = receivedMessageContainer.theNaovaSPLStandardMessage.playerNum;
  theBNTP << receivedMessageContainer;

  snap.lastMessageTeammateNumber = currentTeammate.number;

  receivedMessageContainer.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.bSMB = theBNTP[currentTeammate.number];
  currentTeammate.timeWhenLastPacketReceived = theFrameInfo.time;

  // currentTeammate.theRawGCData = receivedMessageContainer.theNaovaStandardMessage.gameControlData;

  currentTeammate.hasGroundContact = receivedMessageContainer.theNaovaStandardMessage.hasGroundContact;
  currentTeammate.isPenalized = receivedMessageContainer.theNaovaStandardMessage.isPenalized;
  currentTeammate.isUpright = receivedMessageContainer.theNaovaStandardMessage.isUpright;

  // RECEIVE_PARTICLE(SideConfidence);
  RECEIVE_PARTICLE(RobotPose);
  RECEIVE_PARTICLE(BallModel);
  // RECEIVE_PARTICLE(ObstacleModel);
  RECEIVE_PARTICLE(BehaviorStatus);
  // RECEIVE_PARTICLE(SPLStandardBehaviorStatus);
  RECEIVE_PARTICLE(Whistle);
  // RECEIVE_PARTICLE(TeammateRoles);
  RECEIVE_PARTICLE(FieldCoverage);
  // RECEIVE_PARTICLE(RobotHealth);

  receivedMessageContainer.theBHumanArbitraryMessage.queue.handleAllMessages(currentTeammate);
}


bool TeamMessageHandler::sendMessage(){

  if (getRemainingMessages() <= securityMessage) {
    return false;
  }

  // Reset snap.firstMessage for half
  if (theGameInfo.state == STATE_INITIAL) {
    log("RESETTING snap.firstMessage for half", "COMM INFO");
    snap.firstMessage = true;
  }

  // Reset snap.firsMessage when a goal is scored
  if (theGameInfo.state == STATE_READY && snap.lastGameState == STATE_PLAYING) {
    log("RESETTING snap.firstMessage for goal", "COMM INFO");
    snap.firstMessage = true;
    return false;
  }

  // Always send message at begining of both halfs
  if ((theGameInfo.state == STATE_READY || theGameInfo.state == STATE_PLAYING) && snap.firstMessage) { 
    log("Sending first message", "COMM INFO");
    snap.firstMessage = false;
    return true;
  }

  // DON'T send messages if the game is not in playing
  if (theGameInfo.state != STATE_PLAYING){
    return false;
  }

  // Uncomment desired system
  return eventBasedSystem();
  //return timeBasedSystem();
}

bool TeamMessageHandler::eventBasedSystem() {
  // Send message when a whistle is heard
  // if (theGameInfo.state == STATE_SET) {
  //   if (getScoreWhistleSET() >= minScore) {
  //     log("I HEARD A WHISTLE. Sending a message", "COMM INFO");
  //     return true;
  //   } else {
  //     return false;
  //   }
  // }

  adjustScore();

  int scoreRobotMoved = getScoreRobotMoved();
  int scoreBallMoved = getScoreBallMoved();
  int scoreBallDetected = getScoreBallDetected();
  int scoreFallen = getScoreFallen();
  //int scoreWhistle = getScoreWhistle();
  int scorePenalized = getScorePenalized();
  int scoreUnpenalized = getScoreUnpenalized();
  int scoreRoleChanged = getScoreRoleChanged();
  

  int scoreTotal = scoreFallen + scoreBallMoved + scoreBallDetected + scorePenalized + scoreUnpenalized + scoreRobotMoved + scoreRoleChanged; //+ scoreWhistle;

  if (scoreTotal >= minScore && loggingIsEnabled) {

    std::map<std::string, std::string> valuesToLog;
    valuesToLog["theOwnTeamInfo.messageBudget"] = std::to_string(getRemainingMessages());
    valuesToLog["scoreRobotMoved"] = std::to_string(scoreRobotMoved);
    valuesToLog["scoreBallMoved"] = std::to_string(scoreBallMoved);
    valuesToLog["scoreBallDetected"] = std::to_string(scoreBallDetected);
    valuesToLog["scoreFallen"] = std::to_string(scoreFallen);
    valuesToLog["scorePenalized"] = std::to_string(scorePenalized);
    valuesToLog["scoreUnpenalized"] = std::to_string(scoreUnpenalized);
    valuesToLog["scoreRoleChanged"] = std::to_string(scoreRoleChanged);
    //valuesToLog["scoreWhistle"] = std::to_string(scoreWhistle);
    valuesToLog["scoreTotal"] = std::to_string(scoreTotal);
    valuesToLog["minScore"] = std::to_string(minScore);
    valuesToLog["theGameInfo.state"] = std::to_string(theGameInfo.state);
    valuesToLog["nbSentMessages"] = std::to_string(nbSentMessages);
    log(valuesToLog, "MESSAGE CAN BE SENT");
  }

  return scoreTotal >= minScore;
}

bool TeamMessageHandler::timeBasedSystem() {
  if (theFrameInfo.getTimeSince(snap.lastTimeSendMessage) > minTimeBetweenMessages) {
    return true;
  }
  return false;
}

void TeamMessageHandler::log(std::map<std::string, std::string> values, std::string reasonForLogging) {
  if (theOwnTeamInfo.teamNumber == 1) {
    OUTPUT_TEXT("\n\n\n(" << theRobotInfo.number << "): " << reasonForLogging << " ---------------------------------------------------------------");
    OUTPUT_TEXT("(" << theRobotInfo.number << "): currentSystemTime: " << Time::getCurrentSystemTime() << "\n");
    for(auto v : values) {
      OUTPUT_TEXT("(" << theRobotInfo.number << "): " << v.first << ": " << v.second << "\n");
    }
  }
}

void TeamMessageHandler::log(std::string value, std::string reasonForLogging) {
  if (loggingIsEnabled && theOwnTeamInfo.teamNumber == 1) {
    OUTPUT_TEXT("\n\n\n(" << theRobotInfo.number << "): " << reasonForLogging << " ---------------------------------------------------------------");
    OUTPUT_TEXT("(" << theRobotInfo.number << "): " << value);
  }
}

void TeamMessageHandler::adjustScore(){
  // La valeur de theGameInfo.secsInHalf n'est pas correctement attribuee (0 ou 32590?????) 600 = theGameInfo.secsInHalf
  int secsInHalf = 600;
  int remainingSeconds = theGameInfo.firstHalf == 1 ? secsInHalf + theGameInfo.secsRemaining : theGameInfo.secsRemaining;
  int elapsedSeconds = (secsInHalf*2)-remainingSeconds;

  float scoreSlopeDeltaX = (float)((limitMessages*2)/8); // the more we decrease this value, the more drastic the score adjustment will be
  float scoreSlope = (float)(scoreFloor-scoreCeiling)/scoreSlopeDeltaX;
  float targetSentPacketsSlope = (float)((limitMessages)/(secsInHalf*2));

  int expectedNumberOfSentPackets = (int)(targetSentPacketsSlope*elapsedSeconds);
  int actualNumberOfSentPackets = (limitMessages - getRemainingMessages());

  int minScoreTemp = (int)(scoreSlope*(expectedNumberOfSentPackets-actualNumberOfSentPackets) + defaultMinScore);

  // Enforce score threshold lower and upper bounds
  if (minScoreTemp < scoreFloor) {
    minScore = scoreFloor;
  } else if (minScoreTemp > scoreCeiling) {
    minScore = scoreCeiling;
  } else {
    minScore = minScoreTemp;
  }
}


int TeamMessageHandler::getScoreBallMoved(){
  // Score based on the distance between the current ball position and last sent ball position
  Vector2f currentBallPosition = theBallModel.estimate.position;

  // ** peut etre quil serait mieux de calculer le deplacement entre theBallmodel.estimate.position et theBallModel.lastPerception **
  // ** OU calculer le deplacement entre theBallModel.estimate.position et theTeamBallModel.position **
  Vector2f lastBallPosition = snap.lastBallPosition;
  float difference = (currentBallPosition - lastBallPosition).norm();

  if (difference < minBallRadius){
    return 0;
  }

  // Si on voit la balle, mais sa position est tres proche de la derniere position de balle envoyee --> lower score
  int ballDetectedScore = (int)(ballMoved * (difference/(maxBallRadius-minBallRadius)));

  if (loggingIsEnabled) {
    std::map<std::string, std::string> valuesToLog;
    valuesToLog["currentBallPosition.X"] = std::to_string(currentBallPosition.x());
    valuesToLog["currentBallPosition.Y"] = std::to_string(currentBallPosition.y());
    valuesToLog["lastBallPosition.X"] = std::to_string(lastBallPosition.x());
    valuesToLog["lastBallPosition.Y"] = std::to_string(lastBallPosition.y());
    valuesToLog["difference"] = std::to_string(difference);
    valuesToLog["ballDetectedScore"] = std::to_string(ballDetectedScore);
    valuesToLog["theBallPercept.status"] = std::to_string(theBallPercept.status);
    valuesToLog["theBallPercept.confidenceLevel"] = std::to_string(theBallPercept.confidenceLevel);
    valuesToLog["theBallModel.seenPercentage"] = std::to_string(theBallModel.seenPercentage);
    //log(valuesToLog, "BALL MOVEMENT SCORE");
  }

  return ballDetectedScore;
}

int TeamMessageHandler::getScoreBallDetected() {
  if (theBallPercept.status == BallPercept::seen && theFrameInfo.getTimeSince(snap.lastTimeBallDetectedMessage) > minBallDetectedTime) {
    snap.lastTimeBallDetectedMessage = theFrameInfo.time;
    return minScore;
  }
  return 0;
}

int TeamMessageHandler::getScoreRobotMoved(){
  // Comparer la position du robot depuis le dernier envoie 
  Vector2f lastPosition = snap.lastPose;
  Vector2f currentPosition = theRobotPose.translation;

  float difference = ((currentPosition - lastPosition)).norm();

  if (difference < minDistanceRobot) {
    return 0;
  }

  if (loggingIsEnabled) {
    std::map<std::string, std::string> valuesToLog;
    valuesToLog["lastSentPosition.X"] = std::to_string(lastPosition.x());
    valuesToLog["lastSentPosition.Y"] = std::to_string(lastPosition.y());
    valuesToLog["currentPosition.X"] = std::to_string(currentPosition.x());
    valuesToLog["currentPosition.Y"] = std::to_string(currentPosition.y());
    valuesToLog["difference"] = std::to_string(difference);
    //log(valuesToLog, "ROBOT MOVED SCORE");
  }

  // multiplier if robot moved more, bigger score (minDistance->0, maxDistance-> robotMoved)
  int robotMovedScore = (int)(robotMoved * (difference/(maxDistanceRobot-minDistanceRobot)));

  return robotMovedScore;
}

int TeamMessageHandler::getScoreFallen(){
  if (loggingIsEnabled) {
    std::map<std::string, std::string> valuesToLog;
    valuesToLog["FallDownState::State::falling"] = std::to_string(FallDownState::State::falling);
    valuesToLog["FallDownState::State::fallen"] = std::to_string(FallDownState::State::fallen);
    valuesToLog["theFallDownState.state"] = std::to_string(theFallDownState.state);
    valuesToLog["theBallModel.estimate.position.norm()"] = std::to_string(theBallModel.estimate.position.norm());
    //log(valuesToLog, "ROBOT FALLEN SCORE");
  }

  if (theFallDownState.state != FallDownState::State::fallen && theFallDownState.state != FallDownState::State::falling){
    return 0;
  }

  // Verifier la position du robot par rapport a la balle. Donc en fonction de la distance entre le robot et la balle
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
  if (loggingIsEnabled) {
    std::map<std::string, std::string> valuesToLog;
    valuesToLog["theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected)"] = std::to_string(theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected));
    valuesToLog["theWhistle.confidenceOfLastWhistleDetection"] = std::to_string(theWhistle.confidenceOfLastWhistleDetection);
    log(valuesToLog, "ROBOT WHISTLE SCORE");
  }
  if (theFrameInfo.getTimeSince(theWhistle.lastTimeWhistleDetected) <= maxWhistleTime && theWhistle.confidenceOfLastWhistleDetection > minWhistleConfidence) {
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

int TeamMessageHandler::getScoreRoleChanged(){
  if (theFrameInfo.getTimeSince(snap.lastTimeRoleChanged) > minRoleChangedTime && theBehaviorStatus.role != snap.lastMessageRole) {
    snap.lastTimeRoleChanged = theFrameInfo.time;
    return minScore;
  }
  return 0;
}

int TeamMessageHandler::getRemainingMessages(){
  #ifdef TARGET_ROBOT
    return theOwnTeamInfo.messageBudget;
  #endif
  return limitMessages - (nbReceivedMessages + nbSentMessages);
}