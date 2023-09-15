/**
 * @file Processes/Cognition.cpp
 * Implementation of a class that represents a process that receives data from the robot at about 60 Hz.
 */

#include "Tools/Math/Eigen.h" // include first to avoid conflicts between Cabsl defines and some clang headers
#include "Cognition.h" // include second header conflicts on Windows
#include "Modules/Configuration/ConfigurationDataProvider/CognitionConfigurationDataProvider.h"
#include "Modules/Infrastructure/CameraProvider/CameraProvider.h"
#include "Modules/Infrastructure/LogDataProvider/CognitionLogDataProvider.h"
#include "Platform/BHAssert.h"
#include "Platform/Time.h"

#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/NaovaMessage.h"

Cognition::Cognition() :
  Process(theDebugReceiver, theDebugSender),
  theDebugReceiver(this),
  theDebugSender(this),
  theMotionReceiver(this),
  theMotionSender(this),
  theSPLMessageHandler(inTeamMessages, outTeamMessage),
  moduleManager({ModuleBase::cognitionInfrastructure, ModuleBase::communication,
                 ModuleBase::perception, ModuleBase::modeling, ModuleBase::behaviorControl}),
  logger(Logger::LoggedProcess::cognition)
{
  theDebugSender.setSize(5200000, 100000);
  theDebugReceiver.setSize(2800000);
  theMotionSender.moduleManager = theMotionReceiver.moduleManager = &moduleManager;
}

void Cognition::init()
{
#ifdef TARGET_SIM
  theSPLMessageHandler.startLocal(Global::getSettings().teamPort, static_cast<unsigned>(Global::getSettings().playerNumber));
#else
  std::string bcastAddr = UdpComm::getWifiBroadcastAddress();
  theSPLMessageHandler.start(Global::getSettings().teamPort, bcastAddr.c_str());
#endif
  moduleManager.load();
  BH_TRACE_INIT("Cognition");

  // Prepare first frame
  numberOfMessages = theDebugSender.getNumberOfMessages();
  OUTPUT(idProcessBegin, bin, 'c');
}

void Cognition::terminate()
{
  moduleManager.destroy();
  Process::terminate();
}

bool Cognition::main()
{
  // read from team comm udp socket
  static_cast<void>(theSPLMessageHandler.receive());
  if(CognitionLogDataProvider::isFrameDataComplete() && CameraProvider::isFrameDataComplete())
  {

    timingManager.signalProcessStart();
    annotationManager.signalProcessStart();


    BH_TRACE_MSG("before TeamData");
    // push teammate data in our system
    if(Blackboard::getInstance().exists("TeamData") &&
       static_cast<const TeamData&>(Blackboard::getInstance()["TeamData"]).generate.operator bool())
      while(!inTeamMessages.empty())
        static_cast<const TeamData&>(Blackboard::getInstance()["TeamData"]).generate(inTeamMessages.takeBack());
    // Reset coordinate system for debug field drawing
    DECLARE_DEBUG_DRAWING("origin:Reset", "drawingOnField"); // Set the origin to the (0,0,0)

    ORIGIN("origin:Reset", 0.0f, 0.0f, 0.0f);

    STOPWATCH_WITH_PLOT("Cognition") moduleManager.execute();

    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager") OUTPUT(idDrawingManager, bin, Global::getDrawingManager());
    DEBUG_RESPONSE_ONCE("automated requests:DrawingManager3D") OUTPUT(idDrawingManager3D, bin, Global::getDrawingManager3D());
    DEBUG_RESPONSE_ONCE("automated requests:StreamSpecification") OUTPUT(idStreamSpecification, bin, Global::getStreamHandler());

    theMotionSender.timeStamp = Time::getCurrentSystemTime();
    BH_TRACE_MSG("before theMotionSender.send()");
    theMotionSender.send();

    if(Blackboard::getInstance().exists("NaovaMessageOutputGenerator")
       && static_cast<const NaovaMessageOutputGenerator&>(Blackboard::getInstance()["NaovaMessageOutputGenerator"]).generate.operator bool()
       && static_cast<const NaovaMessageOutputGenerator&>(Blackboard::getInstance()["NaovaMessageOutputGenerator"]).sendThisFrame)
    {
      static_cast<const NaovaMessageOutputGenerator&>(Blackboard::getInstance()["NaovaMessageOutputGenerator"]).generate(&outTeamMessage);
      BH_TRACE_MSG("before theTeamHandler.send()");
      // printInfo(static_cast<const NaovaMessageOutputGenerator&>(Blackboard::getInstance()["NaovaMessageOutputGenerator"]));
      theSPLMessageHandler.send();
    }

    timingManager.signalProcessStop();
    logger.execute();
    DEBUG_RESPONSE("timing") timingManager.getData().copyAllMessages(theDebugSender);

    DEBUG_RESPONSE("annotation") annotationManager.getOut().copyAllMessages(theDebugSender);
    annotationManager.clear();

    if(theDebugSender.getNumberOfMessages() > numberOfMessages + 1)
    {
      // Send process finished message
      if(Blackboard::getInstance().exists("CameraInfo") &&
         static_cast<const CameraInfo&>(Blackboard::getInstance()["CameraInfo"]).camera == CameraInfo::lower)
      {
        // lower camera -> process called 'd'
        // Send completion notification
        theDebugSender.patchMessage(numberOfMessages, 0, 'd');
        OUTPUT(idProcessFinished, bin, 'd');
      }
      else
      {
        OUTPUT(idProcessFinished, bin, 'c');
      }
    }
    else
      theDebugSender.removeLastMessage();

    BH_TRACE_MSG("theDebugSender.send()");
    theDebugSender.send();

    // Prepare next frame
    numberOfMessages = theDebugSender.getNumberOfMessages();
    OUTPUT(idProcessBegin, bin, 'c');
  }
  else if(Global::getDebugRequestTable().pollCounter > 0 &&
          --Global::getDebugRequestTable().pollCounter == 0)
    OUTPUT(idDebugResponse, text, "pollingFinished");

  if(Blackboard::getInstance().exists("Image"))
  {
    if(SystemCall::getMode() == SystemCall::physicalRobot)
      setPriority(10);
    Thread::sleep(1);
    BH_TRACE_MSG("before waitForFrameData");
    CameraProvider::waitForFrameData();

    if(SystemCall::getMode() == SystemCall::physicalRobot)
      setPriority(0);
  }
  else
    Thread::sleep(33);


  return SystemCall::getMode() != SystemCall::physicalRobot;
}

bool Cognition::handleMessage(InMessage& message)
{
  BH_TRACE_MSG("before Cognition:handleMessage");
  switch(message.getMessageID())
  {
    case idModuleRequest:
    {
      unsigned timeStamp;
      message.bin >> timeStamp;
      moduleManager.update(message.bin, timeStamp);
      return true;
    }
    default:
      return CognitionLogDataProvider::handleMessage(message) ||
             CognitionConfigurationDataProvider::handleMessage(message) ||
             Process::handleMessage(message);
  }
}

// printInfo(static_cast<const NaovaMessageOutputGenerator&>(Blackboard::getInstance()["NaovaMessageOutputGenerator"])); a utiliser avant envoie de paquet
void Cognition::printInfo(const NaovaMessageOutputGenerator& outTeamMessage){
  fprintf(stderr,
     "-----------------------------------------------------------------------------\n\nNaovaSPLStandardMessage : \n version : %d\n playerNum : %d\n teamNum : %d\n fallen : %d\n pose : %f %f %f\n ball : %f %f\n",
     outTeamMessage.theNaovaSPLStandardMessage.version,
     outTeamMessage.theNaovaSPLStandardMessage.playerNum,
     outTeamMessage.theNaovaSPLStandardMessage.teamNum,
     outTeamMessage.theNaovaSPLStandardMessage.fallen,
     outTeamMessage.theNaovaSPLStandardMessage.pose[0],outTeamMessage.theNaovaSPLStandardMessage.pose[1], outTeamMessage.theNaovaSPLStandardMessage.pose[2],
    //  outTeamMessage.theNaovaSPLStandardMessage.ballAge,
     outTeamMessage.theNaovaSPLStandardMessage.ball[0], outTeamMessage.theNaovaSPLStandardMessage.ball[1]);

  fprintf(stderr,
     "NaovaStandardMessage : \n magicNumber : %d\n ballLastPerceptX : %d\n ballLastPerceptY : %d\n ballLastPerceptY : %d\n ballCovariance : %f %f %f\n robotPoseDeviation: %f\n",
    //  outTeamMessage.theNaovaStandardMessage.version,
     outTeamMessage.theNaovaStandardMessage.magicNumber,
    //  outTeamMessage.theNaovaStandardMessage.ballTimeWhenDisappearedSeenPercentage,
     outTeamMessage.theNaovaStandardMessage.ballLastPerceptX,
     outTeamMessage.theNaovaStandardMessage.ballLastPerceptY,
     outTeamMessage.theNaovaStandardMessage.ballCovariance[0], outTeamMessage.theNaovaStandardMessage.ballCovariance[1], outTeamMessage.theNaovaStandardMessage.ballCovariance[2],
     outTeamMessage.theNaovaStandardMessage.robotPoseDeviation);

  fprintf(stderr,
     " robotPoseValidity : %d\n isPenalized : %s\n isUpright : %s\n hasGroundContact : %s\n timestamp : %d\n",
    //  outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[0], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[1], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[2], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[3], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[4], outTeamMessage.theNaovaStandardMessage.robotPoseCovariance[5],
     outTeamMessage.theNaovaStandardMessage.robotPoseValidity,
     outTeamMessage.theNaovaStandardMessage.isPenalized ? "true" : "false",
     outTeamMessage.theNaovaStandardMessage.isUpright ? "true" : "false",
     outTeamMessage.theNaovaStandardMessage.hasGroundContact ? "true" : "false",
     outTeamMessage.theNaovaStandardMessage.timestamp);
    //  outTeamMessage.theNaovaStandardMessage.keeperIsPlaying ? "true" : "false",
    //  outTeamMessage.theNaovaStandardMessage.passTarget,
    //  outTeamMessage.theNaovaStandardMessage.timeWhenReachBall,
    //  outTeamMessage.theNaovaStandardMessage.timeWhenReachBallStriker);

  fprintf(stderr,
     " timestampLastJumped :  %d\n ballTimeWhenLastSeen : %d\n--------------------------------------------------\n\n",
     outTeamMessage.theNaovaStandardMessage.timestampLastJumped,
     outTeamMessage.theNaovaStandardMessage.ballTimeWhenLastSeen
    //  outTeamMessage.theNaovaStandardMessage.lastTimeWhistleDetected,
    //  outTeamMessage.theNaovaStandardMessage.requestsNTPMessage ? "true" : "false"
    );

  // fprintf(stderr,
  //    " OwnTeamInfo :\n   timestampWhenReceived : %d\n   packetNumber : %d\n   competitionType : %d\n   competitionPhase : %d\n   state : %d\n   setPlay : %d\n   firstHalf : %d\n   kickingTeam : %d\n   gamePhase : %d\n   dropInTeam : %d\n   dropInTime : %d\n   secsRemaining : %d\n   secondaryTime : %d\n   score : %d\n\n",
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.timestampWhenReceived,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.packetNumber,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.competitionTyprintf(stderr,
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
  //   );pe,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.competitionPhase,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.state,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.setPlay,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.firstHalf,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.kickingTeam,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.gamePhase,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.dropInTeam,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.dropInTime,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.secsRemaining,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.secondaryTime,
  //    outTeamMessage.theNaovaStandardMessage.gameControlData.score
  //    );
}

MAKE_PROCESS(Cognition);

// Make sure that two time consuming modules are linked from the Controller library.
#ifdef MACOS

#include "Modules/Perception/BallPerceptors/BallPerceptor.h"
extern Module<BallPerceptor, BallPerceptorBase> theBallPerceptorModule;
auto linkBallPerceptor = &theBallPerceptorModule;

#include "Modules/Perception/ImagePreprocessors/ECImageProvider.h"
extern Module<ECImageProvider, ECImageProviderBase> theECImageProviderModule;
auto linkECImageProvider = &theECImageProviderModule;

#endif
