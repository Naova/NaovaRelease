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
#include <map>
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Infrastructure/ExtendedGameInfo.h"

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

  void update(NaovaMessageOutputGenerator& outputGenerator) override;
  void generateMessage(NaovaMessageOutputGenerator& outputGenerator) const;
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
};
