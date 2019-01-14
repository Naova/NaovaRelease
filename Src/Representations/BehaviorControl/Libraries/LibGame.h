/**
 * @file LibGame.h
 *
 * This file defines a representation that provide information about the course of the game states
 *
 * @author Daniel Krause
 */

#pragma once
#include "Tools/Function.h"
#include "Tools/Streams/AutoStreamable.h"

STREAMABLE(LibGame,
{,
  (int)(0) timeSinceLastPenaltyEnded,
  (unsigned)(0) lastTimeWhenBallWentOut,
  (unsigned)(0) timeWhenLastReadyStarted,
  (unsigned)(0) timeWhenLastSearchedBall,
  (int)(0) timeSinceReadyStarted,
  (unsigned)(0) timeWhenLastSetStarted,
  (int)(0) timeSinceSetStarted,
  (unsigned)(0) timeWhenLastPlayingStarted,
  (int)(0) timeSincePlayingStarted,
  (char) gameStateLastFrame,
  (char) previousGameState,
  (bool)(false) didNotHearWhistleThisTime,
});
