/**
 * @file Tools/Communication/SPLMessageHandler.cpp
 *
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 *
 * based on TeamHandler.cpp authored by
 * @author Thomas Röfer
 */

#include "SPLMessageHandler.h"
#include "Platform/BHAssert.h"
#include "Platform/SystemCall.h"
#include "Platform/Time.h"
#include "Tools/Debugging/DebugDrawings.h"

void SPLMessageHandler::startLocal(int port, unsigned localId)
{
  ASSERT(!this->port);
  this->port = port;
  this->localId = localId;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(false));
  std::string group = SystemCall::getHostAddr();
  group = "239" + group.substr(group.find('.'));
  VERIFY(socket.bind("0.0.0.0", port));
  VERIFY(socket.setTTL(0)); //keep packets off the network. non-standard(?), may work.
  VERIFY(socket.joinMulticast(group.c_str()));
  VERIFY(socket.setTarget(group.c_str(), port));
  socket.setLoopback(true);
}

void SPLMessageHandler::start(int port, const char* subnet)
{
  ASSERT(!this->port);
  this->port = port;

  socket.setBlocking(false);
  VERIFY(socket.setBroadcast(true));
  VERIFY(socket.bind("0.0.0.0", port));
  socket.setTarget(subnet, port);
  socket.setLoopback(false);
}

void SPLMessageHandler::send()
{
  if(!port)
    return;

  socket.write(reinterpret_cast<char*>(&out), SPL_STANDARD_MESSAGE_DATA_SIZE);
}

void SPLMessageHandler::receive()
{
  if(!port)
    return; // not started yet

  int size;
  unsigned remoteIp = 0;

  do
  {
    auto* entry = in.setForward();
    size = localId ? socket.readLocal(reinterpret_cast<char*>(&entry->message), sizeof(entry->message))
                   : socket.read(reinterpret_cast<char*>(&entry->message), sizeof(entry->message), remoteIp);
    if(size < SPL_STANDARD_MESSAGE_DATA_SIZE || size > SPL_STANDARD_MESSAGE_DATA_SIZE)
      in.removeFront();
    else
      entry->timestamp = Time::getCurrentSystemTime();
  }
  while(size > 0);
}
