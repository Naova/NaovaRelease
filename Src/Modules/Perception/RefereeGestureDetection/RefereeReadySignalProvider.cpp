/**
 * @file RefereeReadySignalProvider.cpp
 *
 * This file provides the representation RefereeReadySignal.
 *
 * @author wilht
 */

#include "RefereeReadySignalProvider.h"

MAKE_MODULE(RefereeReadySignalProvider, perception);

using Gesture = RefereePercept::Gesture;

void RefereeReadySignalProvider::update(RefereeReadySignal &theRefereeReadySignal)
{
  theRefereeReadySignal.isConfirmed = false;

  if(theGameInfo.state == STATE_INITIAL)
  {
    theRefereeReadySignal.isDetected = false;
    timeSinceLastInitial = theFrameInfo.time;
  }
  else if(theGameInfo.state == STATE_STANDBY)
  {
    // The robot itself has seen the referee doing the gesture.
    if (theRefereePercept.gesture == Gesture::initialToReady)
      theRefereeReadySignal.isDetected = true;

    // To increase reliability, we wait till enough robots have detected the gesture.
    if (getNumberOfRobotDetections(theRefereeReadySignal) >= minimumNumberOfRobotDetections)
      theRefereeReadySignal.isConfirmed = true;

  }
}

short RefereeReadySignalProvider::getNumberOfRobotDetections(const RefereeReadySignal &theRefereeReadySignal) const
{
  short nbDetections = theRefereeReadySignal.isDetected ? 1 : 0;
  for (auto &teammate : theTeamData.teammates)
  {
    // We want to ignore the packets that comes from before this instance.
    // The offset of 3000
    bool packetStillValid = teammate.timeWhenLastPacketSent > timeSinceLastInitial;
    if (teammate.theRefereeReadySignal.isDetected && packetStillValid)
      ++nbDetections;
    
    if(!packetStillValid && teammate.theRefereeReadySignal.isDetected)
      OUTPUT_TEXT("Invalidating packet: <" << timeSinceLastInitial);
  }

  return nbDetections;
}
