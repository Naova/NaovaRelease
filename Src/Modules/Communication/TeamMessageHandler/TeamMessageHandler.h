/**
 * @file TeamMessageHandler.h
 *
 * Declares a module, that provide the port between SPLStandardMessage
 *          and the B-Human data-systems (Representations)
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#pragma once

#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/TeamBehaviorStatus.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Communication/TeamInfo.h"
#include "Representations/Communication/RobotHadBallContact.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/TeamTalk.h"
#include "Representations/Modeling/FieldCoverage.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/Whistle.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/Perception/FieldFeatures/FieldFeatureOverview.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/GroundContactState.h"
#include "Tools/Module/Module.h"
#include "Representations/Communication/NaovaMessage.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/Communication/BNTP.h"
#include "Tools/Communication/CompressedTeamCommunicationStreams.h"
#include "Tools/Communication/RobotStatus.h"
#include "Tools/Streams/Enum.h"
#include <map>
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"

// BNTP, FieldCoverage, RobotHealth and FieldFeatureOverview cannot be part of this for technical reasons.
#define FOREACH_TEAM_MESSAGE_REPRESENTATION(_) \
  _(BallModel); \
  _(ObstacleModel); \
  _(Whistle); \
  _(BehaviorStatus); \
  _(TeamBehaviorStatus); \
  _(RobotPose); \
  _(RobotStatus); \
  _(RobotHadBallContact); \
  _(RefereeReadySignal);

#define NUMBER_OF_REPRESENTATIONS 9
#define EMPTY_MESSAGE_SIZE 10

MODULE(TeamMessageHandler,
{,
  // for calculations
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  USES(ExtendedGameInfo),
  USES(OwnTeamInfo),
  USES(MotionRequest),
  USES(RawGameInfo),
  USES(TeamData),
  USES(GameInfo),

  // extract data to send
  REQUIRES(FallDownState),
  REQUIRES(GroundContactState),
  USES(RobotInfo),

  // send directly
  USES(BallModel),
  USES(BallPercept),
  USES(BehaviorStatus),
  USES(FieldCoverage),
  REQUIRES(FieldFeatureOverview),
  USES(ObstacleModel),
  USES(RobotHealth),
  USES(RobotPose),
  USES(TeamBehaviorStatus),
  USES(TeamTalk),
  USES(Whistle),
  USES(RobotHadBallContact),
  USES(RefereeReadySignal),

  PROVIDES(NaovaMessageOutputGenerator),
  PROVIDES(TeamData),

  LOADS_PARAMETERS(
  {,
    (int) sendInterval, /**<  Time in ms between two messages that are sent to the teammates */
    (int) networkTimeout, /**< Time in ms after which teammates are considered as unconnected */
    (int) minTimeBetween2RejectSounds, /**< Time in ms after which another sound output is allowed */
    (bool) sendMirroredRobotPose, /**< Whether to send the robot pose mirrored (useful for one vs one demos such that keeper and striker can share their ball positions). */
    (bool) enableCommunication1s, /**<  Whether to send the communication per second or event based */

    (int) limitMessages,
    (int) securityMessage,

    (int) ballMoved,
    (int) robotMoved,
    (int) fallen,
    (int) whistle,
    (int) maxFallenDistance,
    (int) minDistanceRobot,
    (int) maxDistanceRobot,
    (int) minBallRadius,
    (int) maxBallRadius,
    (int) maxWhistleTime,
    (int) minWhistleConfidence,
    (int) minRoleChangedTime,
    (int) minBallDetectedTime,
    (int) scoreCeiling,
    (int) scoreFloor,
    (int) defaultMinScore,
    (int) totalSecsGame,
    (int) minDistanceToSendPose,
  }),
});

/**
 * @class TeamDataSender
 * A modules for sending some representation to teammates
 */
class TeamMessageHandler : public TeamMessageHandlerBase
{
public:
  TeamMessageHandler();

private:
  BNTP theBNTP;
  mutable RobotStatus theRobotStatus;

  CompressedTeamCommunication::TypeRegistry teamCommunicationTypeRegistry;
  const CompressedTeamCommunication::Type* teamMessageType;

  // output stuff
  mutable unsigned timeLastSent = 0;
  
  mutable unsigned int nbSentMessages = 0;
  mutable unsigned int nbReceivedMessages = 0;
  mutable int minScore = defaultMinScore;
  bool refereeDetectedLastState = false;

  void update(NaovaMessageOutputGenerator& outputGenerator) override;
  void generateMessage(NaovaMessageOutputGenerator& outputGenerator);
  void writeMessage(NaovaMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const;
  void log(std::map<std::string, std::string> values, std::string reasonForLogging);
  void log(std::string value, std::string reasonForLogging);


  // input stuff
  struct ReceivedNaovaMessage : public NaovaMessage
  {
    const SynchronizationMeasurementsBuffer* bSMB = nullptr;
    unsigned toLocalTimestamp(unsigned remoteTimestamp) const override
    {
      if(bSMB)
        return bSMB->getRemoteTimeInLocalTime(remoteTimestamp);
      else
        return 0u;
    };

    enum ErrorCode
    {
      //add more parsing errors if there is a need of distinguishing
      parsingError,
      magicNumberDidNotMatch,
      myOwnMessage
    } lastErrorCode;
  } receivedMessageContainer;

  static void regTeamMessage();

  void update(TeamData& teamData) override;
  void maintainBMateList(TeamData& teamData) const;

  unsigned timeWhenLastMimimi = 0;
  bool readSPLStandardMessage(const SPLStandardMessageBufferEntry* const m);
  Teammate& getBMate(TeamData& teamData) const;
  void parseMessageIntoBMate(Teammate& bMate);

  int getRemainingMessages();
  bool sendMessage();

  // Function for score system
  int getScoreRobotMoved();
  int getScoreBallMoved();
  int getScoreBallDetected();
  int getScoreFallen();
  int getScorePenalized();
  int getScoreUnpenalized();
  int getScoreWhistle();
  int getScoreWhistleSET();
  int getScoreRobotBallContact();
  int getScoreRefereeReadySignal();
  int getScoreKickoff();
  void adjustScore();

  // struct represente data state before sent
  struct Snap{
    bool firstMessage = true;
    Vector2f lastPose;
    uint32_t lastTimeSendMessage = 0;
    uint8_t lastPenaltyState;
    PlayerRole::RoleType lastMessageRole;
    int lastMessageTeammateNumber;
    Vector2f lastBallPosition;
    int lastTimeBallDetectedMessage;
  } snap;

  // Message type
  ENUM(messageRepresentation, //Represente l'ordre de priorite des representations
  {,
    robotStatusIndex,
    robotPoseIndex,
    refereeReadySignalIndex,
    ballModelIndex,
    robotHadBallContactIndex,
    obstacleModelIndex,
    whistleIndex,
    behaviorStatusIndex,
    teamBehaviorStatusIndex,
  });

  mutable uint8_t lastFrameState;
  mutable uint16_t currentMessageType = 0;
  mutable uint8_t maxObstacles;
  mutable uint8_t messageCount;
  std::string representationOptions[NUMBER_OF_REPRESENTATIONS] = {"  theRobotStatus: RobotStatus\n", "  theRobotPose: RobotPose\n", "  theRefereeReadySignal: RefereeReadySignal\n", "  theBallModel: BallModel\n", " theRobotHadBallContact: RobotHadBallContact\n", "  theObstacleModel: ObstacleModel\n", "  theWhistle: Whistle\n", "  theBehaviorStatus: BehaviorStatus\n", "  theTeamBehaviorStatus: TeamBehaviorStatus\n"}; 
  int representationSizes[NUMBER_OF_REPRESENTATIONS] = {3,30,1,33,1,18,3,5,8}; //Taille en bit

  void changeMessageType();
  bool getBitValue(uint16_t compressed_header, int index);
  void setBitValue(uint16_t& compressed_header, int index, bool value);
  void addToMessage(int& messageSize, int representationIndex);
};
