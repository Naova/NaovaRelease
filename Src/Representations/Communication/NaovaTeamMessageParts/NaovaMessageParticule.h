/**
 * @file NaovaMessageParticle.h
 * ref BHumanMessageParticule.h de Jesse Richter
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once


#include <functional>
#include "Tools/Streams/AutoStreamable.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Platform/BHAssert.h"
#include "Representations/Communication/NaovaMessage.h"

template<MessageID ID>
struct NaovaMessageParticule
{
  static MessageID id() { return ID; }

  virtual void operator>>(NaovaMessage& m) const = 0;
  virtual void operator<<(const NaovaMessage& m) { ; };
  virtual bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned u)>& toLocalTimestamp) { return false; };
};

template<MessageID ID>
In& operator >> (In& in, NaovaMessageParticule<ID>& particle)
{
  return in >> dynamic_cast<Streamable&>(particle);
}

template<MessageID ID>
Out& operator<<(Out& out, const NaovaMessageParticule<ID>& particle)
{
  return out << dynamic_cast<const Streamable&>(particle);
}

template<MessageID ID>
struct PureNaovaArbitraryMessageParticle : public NaovaMessageParticule<ID>
{
  void operator >> (NaovaMessage& m) const override
  {
    m.theBHumanArbitraryMessage.queue.out.bin << *this;
    m.theBHumanArbitraryMessage.queue.out.finishMessage(this->id());
  }

  void operator<<(const NaovaMessage& m) override { ; }

  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned u)>& toLocalTimestamp) override
  {
    ASSERT(m.getMessageID() == this->id());
    m.bin >> *this;
    return true;
  }
};
