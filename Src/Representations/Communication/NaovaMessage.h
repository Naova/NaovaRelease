#pragma once

#include "Tools/Communication/RoboCupGameControlData.h"
#include "Tools/Communication/NaovaTeamMessageParts/NaovaStandardMessage.h"
#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(NaovaMessage,
{
    virtual unsigned toLocalTimestamp(unsigned remoteTimestamp) const,

    (unsigned) timestamp,
    (NaovaStandardMessage) theNaovaStandardMessage,
});

inline unsigned NaovaMessage::toLocalTimestamp(unsigned) const { return 0u; }

STREAMABLE_WITH_BASE(NaovaMessageOutputGenerator, NaovaMessage,
{
  FUNCTION(void(RoboCup::SPLStandardMessage* const m)) generate,

  (unsigned)(0) sentMessages,  //< count of sent messages
  (bool)(false) sendThisFrame, //< should send this frame -> is allowed to send this frame considering the spl rules
});