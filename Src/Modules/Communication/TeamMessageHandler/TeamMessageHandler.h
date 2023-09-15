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
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/BehaviorControl/Role.h"
#include "Representations/BehaviorControl/SPLStandardBehaviorStatus.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
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
#include "Representations/Communication/TeamData.h"
#include "Tools/Communication/BNTP.h"
#include "Representations/Communication/NaovaMessage.h"

#include <map>

MODULE(TeamMessageHandler,
{,
  // v- using for calculations
  REQUIRES(FrameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(GameInfo),
  USES(OwnTeamInfo),
  USES(MotionRequest),

  // v- using for teamout
  USES(FallDownState),
  USES(GroundContactState),
  USES(JointAngles),
  USES(RawGameInfo),
  USES(RobotInfo),

  // v- directly sliding into teamout
  USES(BallModel),
  USES(BallPercept),
  USES(BehaviorStatus),
  USES(FieldCoverage),
  USES(FieldFeatureOverview),
  USES(ObstacleModel),
  USES(RobotHealth),
  USES(RobotPose),
  USES(SideConfidence),
  USES(SPLStandardBehaviorStatus),
  USES(TeamData),
  USES(TeammateRoles),
  USES(Whistle),

  PROVIDES(NaovaMessageOutputGenerator),
  PROVIDES(TeamData),

  LOADS_PARAMETERS(
  {,
    (int) minTimeBetween2RejectSounds, /*< Time in ms after which another sound output is allowed */
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
    (int) minTimeBetweenMessages,
    (int) scoreCeiling,
    (int) scoreFloor,
    (int) defaultMinScore,
  }),
});

//#define SELF_TEST_TeamMessageHandler

/**
 * @class TeamDataSender
 * A modules for sending some representation to teammates
 */
class TeamMessageHandler : public TeamMessageHandlerBase
{
public:
  TeamMessageHandler() : TeamMessageHandlerBase(), theBNTP(theFrameInfo, theRobotInfo) {
    limitMessages = abs(limitMessages);
    securityMessage = abs(securityMessage);
  }

private:
  BNTP theBNTP;

  // v- output stuff
  mutable unsigned timeLastSent = 0;
  mutable unsigned int nbSentMessages = 0;
  mutable unsigned int nbReceivedMessages = 0;
  mutable int minScore = defaultMinScore;
  bool loggingIsEnabled = false;

  void update(NaovaMessageOutputGenerator& outputGenerator);
  void generateMessage(NaovaMessageOutputGenerator& outputGenerator) const;
  void writeMessage(NaovaMessageOutputGenerator& outputGenerator, RoboCup::SPLStandardMessage* const m) const;
  void log(std::map<std::string, std::string> values, std::string reasonForLogging);
  void log(std::string value, std::string reasonForLogging);

  // v- input stuff
  struct ReceivedBHumanMessage : public NaovaMessage
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

  void update(TeamData& teamData);
  void maintainBMateList(TeamData& teamData) const;

  unsigned timeWhenLastMimimi = 0;
  bool readSPLStandardMessage(const RoboCup::SPLStandardMessage* const m);
  Teammate& getBMate(TeamData& teamData) const;
  void parseMessageIntoBMate(Teammate& bMate);
  int getRemainingMessages();
  bool sendMessage(); 
  bool eventBasedSystem(); // ici pour verifier si on envoie ou non (avec systeme de score)
  bool timeBasedSystem();  // ici pour envoyer un message par seconde par robot

  // Function for score system
  // int getScoreBallDetected();
  int getScoreRobotMoved();
  int getScoreBallMoved();
  int getScoreBallDetected();
  int getScoreFallen();
  int getScorePenalized();
  int getScoreUnpenalized();
  int getScoreRoleChanged();
  int getScoreWhistle();
  int getScoreWhistleSET();
  void adjustScore();

  // struct represente data state before sent
  struct Snap{
    bool firstMessage = true;
    uint8_t lastGameState;
    Vector2f lastPose;
    uint32_t lastTimeSendMessage = 0;
    int lastTimeRoleChanged = 0;
    uint8_t lastPenaltyState;
    Role::RoleType lastMessageRole;
    int lastMessageTeammateNumber;
    Vector2f lastBallPosition;
    int lastTimeBallDetectedMessage;
  } snap;
};
