#pragma once

#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"
#include "NaovaTeamMessageParts/NaovaSPLStandardMessage.h"
#include "NaovaTeamMessageParts/NaovaStandardMessage.h"
#include "BHumanTeamMessageParts/BHumanArbitraryMessage.h"

STREAMABLE(NaovaMessage,
{
    virtual unsigned toLocalTimestamp(unsigned remoteTimestamp) const { return 0u; },
    (NaovaSPLStandardMessage) theNaovaSPLStandardMessage,
    (NaovaStandardMessage) theNaovaStandardMessage,
    (BHumanArbitraryMessage) theBHumanArbitraryMessage,
});

STREAMABLE_WITH_BASE(NaovaMessageOutputGenerator, NaovaMessage,
{
  FUNCTION(void(RoboCup::SPLStandardMessage* const m)) generate,

  (unsigned)(0) sentMessages,  //< count of sent messages
  (bool)(false) sendThisFrame, //< should send this frame -> is allowed to send this frame considering the spl rules
});