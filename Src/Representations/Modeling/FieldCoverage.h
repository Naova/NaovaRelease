/**
 * @file FieldCoverage.h
 * @author Andreas Stolpmann
 */

#pragma once

#include "Tools/Streams/AutoStreamable.h"
#include "Tools/Math/Eigen.h"
#include <vector>
#include "Representations/Communication/NaovaTeamMessageParts/NaovaMessageParticule.h"

STREAMABLE(FieldCoverage, COMMA public NaovaMessageParticule<idFieldCoverage>
{
  /** NaovaMessageParticle functions */
  void operator >> (NaovaMessage& m) const override;
  void operator << (const NaovaMessage& m) override;
  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp) override;

  STREAMABLE(GridLine,
  {,
    (int) y,
    (std::vector<unsigned>)() timestamps,
  }),

  (int)(0) lineToSendNext,
  (std::vector<GridLine>) lines,
});
