/**
 * @file RefereeReadySignalProvider.h
 *
 * This file declares a module that provides the representation RefereeReadySignal.
 *
 * @author wilht
 */

#pragma once

#include "Tools/Module/Module.h"
#include "Representations/Perception/RefereePercept/RefereePercept.h"
#include "Representations/Perception/RefereePercept/RefereeReadySignal.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"

MODULE(RefereeReadySignalProvider,
{,
  USES(GameInfo),
  REQUIRES(FrameInfo),
  REQUIRES(TeamData),
  REQUIRES(RefereePercept),
  PROVIDES(RefereeReadySignal),
  LOADS_PARAMETERS(
  {,
    (short) minimumNumberOfRobotDetections,
  }),
});

class RefereeReadySignalProvider : public RefereeReadySignalProviderBase
{
  /**
   * This method is called when the representation provided needs to be updated.
   * @param theRefereeReadySignal The representation updated.
   */
  void update(RefereeReadySignal& theRefereeReadySignal) override;

  short getNumberOfRobotDetections(const RefereeReadySignal &theRefereeReadySignal) const;

  unsigned timeSinceLastInitial;
  unsigned timeSinceInStandby;
  bool alreadyInStandby = false;
  bool lastDetected = false;
  bool lastConfirmed = false;
};
