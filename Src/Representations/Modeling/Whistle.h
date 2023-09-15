/**
 * @file Whistle.h
 *
 * Identified whistle sound
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Representations/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"
#include "Platform/Time.h"

STREAMABLE(Whistle, COMMA public NaovaMessageParticule<idWhistle>
{
  /** NaovaMessageParticle functions */
  void operator >> (NaovaMessage& m) const override;
  void operator << (const NaovaMessage& m) override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override,

  (int)(0)         confidenceOfLastWhistleDetection, /**< Confidence based on hearing capability */
  (unsigned int)(0) lastTimeWhistleDetected,          /**< Timestamp */
  (unsigned int)(0) lastTimeOfIncomingSound,          /**< The last point of time when the robot received audio data */
  (std::string)("") whistleName,                      /**< Name of the last detected whistle */
});

inline void Whistle::operator >> (NaovaMessage& m) const
{
  m.theNaovaStandardMessage.lastTimeWhistleDetected = lastTimeWhistleDetected;
  m.theNaovaStandardMessage.confidenceOfLastWhistleDetection = Naova::HearingConfidence(confidenceOfLastWhistleDetection < 0 ? static_cast<int>(-1) : (int)confidenceOfLastWhistleDetection);

  m.theBHumanArbitraryMessage.queue.out.bin << lastTimeOfIncomingSound;
  m.theBHumanArbitraryMessage.queue.out.bin << whistleName;
  m.theBHumanArbitraryMessage.queue.out.finishMessage(id());
}

inline void Whistle::operator << (const NaovaMessage& m)
{
  lastTimeWhistleDetected = m.toLocalTimestamp(m.theNaovaStandardMessage.lastTimeWhistleDetected);
  if(static_cast<char>(m.theNaovaStandardMessage.confidenceOfLastWhistleDetection) == 1){
    confidenceOfLastWhistleDetection = 33;
  }
  else if(static_cast<char>(m.theNaovaStandardMessage.confidenceOfLastWhistleDetection) == 2){
    confidenceOfLastWhistleDetection = 66;
  }
  else if(static_cast<char>(m.theNaovaStandardMessage.confidenceOfLastWhistleDetection) == 3){
    confidenceOfLastWhistleDetection = 100;
  }
  else {
    confidenceOfLastWhistleDetection = static_cast<char>(m.theNaovaStandardMessage.confidenceOfLastWhistleDetection);
  }
  lastTimeOfIncomingSound = Time::getCurrentSystemTime(); // does not matter anyway...
}

inline bool Whistle::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());
  m.bin >> lastTimeOfIncomingSound;
  lastTimeOfIncomingSound = toLocalTimestamp(lastTimeOfIncomingSound);
  m.bin >> whistleName;
  return true;
}
