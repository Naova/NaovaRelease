/**
 * @file Tools/BNTP.cpp
 *
 * Representations and functions for time synchronisation inside
 * the team. Implementation of parts of the Network Time Protocol.
 *
 * @author <a href="mailto:tlaue@uni-bremen.de">Tim Laue</a>
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#include "BNTP.h"
#include "Platform/Time.h"

void BNTP::operator >> (NaovaMessage& m) const
{
  // m.theNaovaStandardMessage.ntpMessages.clear();
  // Process incoming NTP requests:
  while(!receivedNTPRequests.empty())
  {
    const BNTPRequest& request = receivedNTPRequests.back();
    Naova::BNTPMessage bntpMessage;
    bntpMessage.receiver = request.sender;
    bntpMessage.requestOrigination = request.origination;
    bntpMessage.requestReceipt = request.receipt;
    // m.theNaovaStandardMessage.ntpMessages.emplace_back(bntpMessage);

    const_cast<RingBuffer<BNTPRequest, MAX_NUM_OF_NTP_PACKAGES>&>(receivedNTPRequests).pop_back();
  }

  // Send NTP requests to teammates?
  // if((m.theNaovaStandardMessage.requestsNTPMessage = theFrameInfo.time - lastNTPRequestSent >= NTP_REQUEST_INTERVAL))
  //   const_cast<unsigned&>(lastNTPRequestSent) = theFrameInfo.time;
}

void BNTP::operator<<(const NaovaMessage& m)
{
  const unsigned receiveTimeStamp = Time::getCurrentSystemTime();

  // if(m.theNaovaStandardMessage.requestsNTPMessage)
  // {
  //   BNTPRequest ntpRequest;
  //   ntpRequest.sender = m.theNaovaSPLStandardMessage.playerNum;
  //   ntpRequest.origination = m.theNaovaStandardMessage.timestamp;
  //   ntpRequest.receipt = receiveTimeStamp;
  //   receivedNTPRequests.push_front(ntpRequest);
  // }

  // for(auto itr = m.theNaovaStandardMessage.ntpMessages.cbegin(); itr != m.theNaovaStandardMessage.ntpMessages.cend(); ++itr)
  // {
  //   //Check, if this response belongs to an own request:
  //   if(itr->receiver == theRobotInfo.number)
  //   {
  //     SynchronizationMeasurementsBuffer::SynchronizationMeasurement t;
  //     t.roundTrip = int(receiveTimeStamp - itr->requestOrigination) - int(m.theNaovaStandardMessage.timestamp - itr->requestReceipt);
  //     t.offset = int(itr->requestReceipt - itr->requestOrigination + m.theNaovaStandardMessage.timestamp - receiveTimeStamp) / 2;
  //     timeSyncBuffers[m.theNaovaSPLStandardMessage.playerNum].add(t);
  //   }
  // }
}

void SynchronizationMeasurementsBuffer::add(const SynchronizationMeasurement& s)
{
  buffer.push_front(s);

  bestOffset = buffer[0].offset;
  int shortestRoundTrip = buffer[0].roundTrip;
  for(size_t i = 1; i < buffer.size(); ++i)
    if(buffer[i].roundTrip < shortestRoundTrip)
    {
      shortestRoundTrip = buffer[i].roundTrip;
      bestOffset = buffer[i].offset;
    }
}
