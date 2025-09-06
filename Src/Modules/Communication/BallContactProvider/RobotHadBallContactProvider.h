/**
 * @file RobotHadBallContactProvider.h
 *
 * This file declares a module that provides the RobotHadBallContact representation.
 */

#pragma once

#include "Representations/Communication/RobotHadBallContact.h"
#include "Representations/Communication/GameInfo.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Tools/Module/Module.h"

MODULE(RobotHadBallContactProvider,
{,
  REQUIRES(FrameInfo),
  REQUIRES(GameInfo),
  REQUIRES(MotionInfo),
  REQUIRES(TeamData),
  PROVIDES(RobotHadBallContact),
});

class RobotHadBallContactProvider : public RobotHadBallContactProviderBase
{
  uint8_t lastSetPlay = SET_PLAY_NONE; /**< The last set play. */

  public:
    RobotHadBallContactProvider() = default;
    ~RobotHadBallContactProvider() = default;

    /**
     * This method is called when the representation provided needs to be updated.
     * @param RobotHadBallContact The representation updated.
     */
    void update(RobotHadBallContact& ballContact);
    void resetContacts(RobotHadBallContact& ballContact);
};
