/**
 * @file Modules/Infrastructure/NaoProvider.h
 * The file declares a module that provides information from the Nao via DCM.
 * @author <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#pragma once

#ifdef TARGET_ROBOT
#include "Platform/Nao/NaoBody.h"
#endif
#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/JointLimits.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JointRequest.h"
#include "Representations/Infrastructure/LEDRequest.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Infrastructure/SensorData/FsrSensorData.h"
#include "Representations/Infrastructure/SensorData/InertialSensorData.h"
#include "Representations/Infrastructure/SensorData/JointSensorData.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Infrastructure/TeamInfo.h"
#include "Tools/Module/Module.h"
// #include "Tools/Communication/UdpComm.h"


MODULE(NaoProvider,
{,
  REQUIRES(JointCalibration),
  REQUIRES(JointLimits),
  REQUIRES(LEDRequest),

  PROVIDES(FrameInfo),
  REQUIRES(FrameInfo),

  PROVIDES(FsrSensorData),
  PROVIDES(InertialSensorData),
  PROVIDES(JointSensorData),
  PROVIDES(KeyStates),
  PROVIDES(OpponentTeamInfo),
  PROVIDES(OwnTeamInfo),
  PROVIDES(RawGameInfo),
  PROVIDES(RobotInfo),
  USES(RobotInfo),
  PROVIDES(SystemSensorData),
  USES(JointRequest), // Will be accessed in send()
  DEFINES_PARAMETERS(
    {,
      (int)(3000) timeChestButtonPressedUntilShutdown, /**< Time the chest button must be pressed until shutdown (in ms). */
      (int)(5000) timeBetweenBatteryLevelUpdates, /**< Time between writing updates the battery level to a file (in ms). */
      // (int)(2000) gameControllerTimeout, /**< Connected to GameController when packet was received within this period of time (in ms). */
      // (int)(1000) aliveDelay, /**< Send an alive signal in this interval of ms. */
      // (int)(30) buttonDelay, /**< Button state changes are ignored when happening in less than this period (in ms). */
      // (int)(3000) chestButtonPressDuration, /**< Chest button state changes are ignored when happening in more than this period (in ms). */
      // (int)(300) chestButtonTimeout, /**< Accept chest button press when chest button was not pressed again within this period (in ms). */

    }
  ),
});

#ifdef TARGET_ROBOT

/**
 * @class NaoProvider
 * A module that provides information from the Nao.
 */
class NaoProvider : public NaoProviderBase
{
private:
  static thread_local NaoProvider* theInstance; /**< The only instance of this module. */

  NaoBody naoBody;
  int socket;
  unsigned char receivedPacket[896]; /**< The last packet received from LoLA. */
  unsigned char packetToSend[1000]; /**< The packet to send to LoLA. */
  size_t packetToSendSize; /**< The size of the packet to send. */
  
  RoboCup::RoboCupGameControlData gameControlData; /**< The last game control data received. */
  unsigned gameControlTimeStamp = 0; /**< The time when the last gameControlData was received (kind of). */
  

  std::array<std::array<const unsigned char*, FsrSensors::numOfFsrSensors>, Legs::numOfLegs> fsrs; /**< The addresses of fsr data inside the receivedPacket. */
  std::array<const unsigned char*, 3> gyros; /**< The addresses of gyro data inside receivedPacket. */
  std::array<const unsigned char*, 3> accs; /**< The addresses of accelerometer data inside receivedPacket. */
  std::array<const unsigned char*, 3> torsoAngles; /**< The addresses of torso angle data inside receivedPacket. */
  std::array<const unsigned char*, Joints::numOfJoints> jointAngles; /**< The addresses of joint angle data inside receivedPacket. */
  std::array<const unsigned char*, Joints::numOfJoints> jointCurrents; /**< The addresses of joint current data inside receivedPacket. */
  std::array<const unsigned char*, Joints::numOfJoints> jointTemperatures; /**< The addresses of joint temperature data inside receivedPacket. */
  std::array<const unsigned char*, Joints::numOfJoints> jointStatuses; /**< The addresses of joint status data inside receivedPacket. */
  std::array<const unsigned char*, KeyStates::numOfKeys> keys; /**< The addresses of key state data inside receivedPacket. */
  const unsigned char* batteryLevel = nullptr; /**< The address of battery level data inside receivedPacket. */
  const unsigned char* batteryCurrent = nullptr; /**< The address of battery current data inside receivedPacket. */
  const unsigned char* batteryTemperature = nullptr; /**< The address of battery temperature data inside receivedPacket. */
  const unsigned char* batteryCharging = nullptr; /**< The address of battery charging state data inside receivedPacket. */
  std::array<unsigned char*, Joints::numOfJoints> jointRequests; /**< The addresses of joint request data inside packetToSend. */
  std::array<unsigned char*, Joints::numOfJoints> jointStiffnesses; /**< The addresses of joint stiffness data inside packetToSend. */
  std::array<unsigned char*, LEDRequest::numOfLEDs> leds; /**< The addresses of led data inside packetToSend. */

  unsigned timeWhenPacketReceived = 0; /**< The time when the last packet was received. */
  unsigned timeWhenChestButtonUnpressed = 0; /**< The last time the chest buttom was not pressed. */
  unsigned timeWhenBatteryLevelWritten = 0; /**< The last time the battery level was written to a file. */

  static const Joints::Joint jointMappings[Joints::numOfJoints - 1]; /**< Mappings from LoLA's joint indices to B-Human's joint indices. */
  static const KeyStates::Key keyMappings[KeyStates::numOfKeys]; /**< Mappings from LoLA's touch indices to B-Human's key indices. */
  static const LEDRequest::LED leftEyeMappings[LEDRequest::faceRightRed0Deg - LEDRequest::faceLeftRed0Deg]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED rightEyeMappings[LEDRequest::earsLeft0Deg - LEDRequest::faceRightRed0Deg]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED leftEarMappings[LEDRequest::earsRight0Deg - LEDRequest::earsLeft0Deg]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED rightEarMappings[LEDRequest::chestRed - LEDRequest::earsRight0Deg]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED chestMappings[LEDRequest::headLedRearLeft0 - LEDRequest::chestRed]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED skullMappings[LEDRequest::footLeftRed - LEDRequest::headLedRearLeft0]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED leftFootMappings[LEDRequest::footRightRed - LEDRequest::footLeftRed]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */
  static const LEDRequest::LED rightFootMappings[LEDRequest::numOfLEDs - LEDRequest::footRightRed]; /**< Mappings from LoLA's LED indices to B-Human's LED indices. */

  // UdpComm socketGameController; /**< The socket to communicate with the GameController. */
  // RoboCup::RoboCupGameControlData gameCtrlData; /**< The local copy of the GameController packet. */
  // bool previousChestButtonPressed = false; /**< Whether the chest button was pressed during the previous cycle. */
  // bool previousLeftFootButtonPressed = false; /**< Whether the left foot bumper was pressed during the previous cycle. */
  // bool previousRightFootButtonPressed = false; /**< Whether the right foot bumper was pressed during the previous cycle. */
  // unsigned whenChestButtonStateChanged = 0; /**< When last state change of the chest button occured. */
  // unsigned whenChestButtonPressed = 0; /**< When the chest button was pressed. */
  // unsigned whenChestButtonReleased = 0;/**< When the chest button was released. */
  // unsigned whenLeftFootButtonStateChanged = 0; /**< When last state change of the left foot bumper occured. */
  // unsigned whenRightFootButtonStateChanged = 0; /**< When last state change of the right foot bumper occured. */
  // unsigned whenPacketWasReceived = 0; /**< When the last GameController packet was received. */
  // unsigned whenPacketWasSent = 0; /**< When the last return packet was sent to the GameController. */
  // int chestButtonPressCounter = 0; /**< Counter for pressing the chest button*/


public:
  NaoProvider();
  ~NaoProvider();

  /** The method is called by process Motion to send the requests to the Nao. */
  static void finishFrame();
  static void waitForFrameData();

private:
  void update(FrameInfo& frameInfo);
  void update(FsrSensorData& fsrSensorData);
  void update(InertialSensorData& inertialSensorData);
  void update(JointSensorData& jointSensorData);
  void update(KeyStates& keyStates);
  void update(OpponentTeamInfo& opponentTeamInfo);
  void update(OwnTeamInfo& ownTeamInfo);
  void update(RawGameInfo& rawGameInfo);
  void update(RobotInfo& robotInfo);
  void update(SystemSensorData& systemSensorData);

  /** The function sends a command to the Nao. */
  void sendPacket();

  void receivePacket();

  void writeLEDs(const std::string category, const LEDRequest::LED* ledMapping, int numOfLEDs, unsigned char*& p);


  // /**
  //  * Receives a packet from the GameController.
  //  * Packets are only accepted when the team number is know (nonzero) and
  //  * they are addressed to this team.
  //  */
  // bool receive();

  // /** Sends the alive message to the GameController. */
  // bool sendAliveMessage();

  // /** Handle the official button interface. */
  // void handleButtons();
};

#else
//TARGET_ROBOT not defined here (Simulator).

class NaoProvider : public NaoProviderBase
{
  // /**
  //  * Receives a packet from the GameController.
  //  * Packets are only accepted when the team number is know (nonzero) and
  //  * they are addressed to this team.
  //  */
  // bool receive();

  // /** Sends the alive message to the GameController. */
  // bool sendAliveMessage();

  // /** Handle the official button interface. */
  // void handleButtons();

private:
  void update(FrameInfo& frameInfo) {}
  void update(FsrSensorData& fsrSensorData) {}
  void update(InertialSensorData& inertialSensorData) {}
  void update(JointSensorData& jointSensorData) {}
  void update(KeyStates& keyStates) {}
  void update(OpponentTeamInfo& opponentTeamInfo) {}
  void update(OwnTeamInfo& ownTeamInfo) {}
  void update(RawGameInfo& rawGameInfo) {}
  void update(RobotInfo& robotInfo) {}
  void update(SystemSensorData& systemSensorData) {}
  void sendPacket();

public:
  static void finishFrame() {}
  static void waitForFrameData() {}
};

#endif
