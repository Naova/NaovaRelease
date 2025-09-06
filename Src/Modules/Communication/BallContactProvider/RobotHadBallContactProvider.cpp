/**
 * @file RobotHadBallContactProvider.cpp
 * 
 * This file implements an implementation of the RobotHadBallContactProvider module.
 */

#include "RobotHadBallContactProvider.h"

MAKE_MODULE(RobotHadBallContactProvider, communication);

void RobotHadBallContactProvider::update(RobotHadBallContact& ballContact)
{
    if (theGameInfo.setPlay == SET_PLAY_PENALTY_KICK){
        ballContact.hadContact = true;
    }
    else if (theGameInfo.state == STATE_SET){
        resetContacts(ballContact);
    }
    else if (lastSetPlay == SET_PLAY_NONE && theGameInfo.setPlay != SET_PLAY_NONE){
        resetContacts(ballContact);
    }
    else if (lastSetPlay == SET_PLAY_PENALTY_KICK && theGameInfo.setPlay == SET_PLAY_NONE){
        resetContacts(ballContact);
    }
    else{
        ballContact.hadContact = theMotionInfo.lastKickTimestamp>ballContact.timeLastKickoff;
    }
    lastSetPlay = theGameInfo.setPlay;
}

void RobotHadBallContactProvider::resetContacts(RobotHadBallContact& ballContact)
{
    ballContact.timeLastKickoff = theFrameInfo.time;
    ballContact.hadContact = false;
}