/**
 * @file NaovaMessageParticle.h
 * ref BHumanMessageParticule.h de Jesse Richter
 * @author <A href="mailto:jesse@tzi.de">Jesse Richter-Klug</A>
 */

#pragma once


#include <functional>
#include "Tools/Streams/Streamable.h"
#include "Tools/MessageQueue/InMessage.h"
#include "Tools/MessageQueue/MessageIDs.h"
#include "Platform/BHAssert.h"
#include "Representations/Communication/NaovaMessage.h"

template<MessageID ID>
struct NaovaMessageParticule
{
  static MessageID id() { return ID; }

  virtual void operator>>(NaovaMessage&) const = 0;
  virtual void operator<<(const NaovaMessage&) { ; };
  virtual bool handleArbitraryMessage(InMessage&, const std::function<unsigned(unsigned u)>&) { return false; };
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
  void operator >> (NaovaMessage&) const override
  {
  }

  void operator<<(const NaovaMessage&) override {}

  bool handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned u)>&) override
  {
    ASSERT(m.getMessageID() == this->id());
    m.bin >> *this;
    return true;
  }
};

template<typename Message>
struct BHumanCompressedMessageParticle : public NaovaMessageParticule<undefined>
{
  BHumanCompressedMessageParticle() :
    _typeName("the" + TypeRegistry::demangle(typeid(Message).name()))
  {}

  void operator>>(NaovaMessage& m) const override
  {
    Streaming::streamIt(*m.theNaovaStandardMessage.out, _typeName.c_str(), *this);
  }

  void operator<<(const NaovaMessage& m) override
  {
    Streaming::streamIt(*m.theNaovaStandardMessage.in, _typeName.c_str(), *this);
  }

private:
  std::string _typeName;
};