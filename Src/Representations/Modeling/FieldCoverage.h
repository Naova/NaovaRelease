/**
 * @file FieldCoverage.h
 *
 * Declaration to send information about the field coverage.
 *
 * @author Nicole Schrader
 */

#pragma once

#include "Representations/Infrastructure/FrameInfo.h"
#include "Tools/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Streams/AutoStreamable.h"
#include <vector>

STREAMABLE(FieldCoverage, COMMA public NaovaMessageParticule<idFieldCoverage>
{
  /** NaovaMessageParticle functions */
  void operator>>(NaovaMessage& m) const override;
  void operator<<(const NaovaMessage& m) override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  STREAMABLE(GridLine,
  {,
    (int) y,
    (std::vector<unsigned>)() timestamps,
  }),

  (std::array<GridLine, 12>) lines,
});
