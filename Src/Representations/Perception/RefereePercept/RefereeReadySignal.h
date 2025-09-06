#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"

STREAMABLE(RefereeReadySignal, COMMA public BHumanCompressedMessageParticle<RefereeReadySignal>
{,
  (bool)(false) isDetected, /**< If the robot itself has detected the initialToReady gesture from the referee. */
  (bool)(false) isConfirmed, /**< If a consensus has been reached. When set to true, it will trigger the transition to ready. Not sent in the team data. */
});
