/**
 * @file FieldFeatureOverview.cpp
 * @author <a href="mailto:jesse@tzi.de">Jesse Richter-Klug</a>
 */

#include "FieldFeatureOverview.h"
#include "Tools/Debugging/DebugDrawings.h"
#include "Representations/Infrastructure/FrameInfo.h"

#define PLOT_SINGE_TSL(name) \
  PLOT("representation:FieldFeatureOverview:timeSinceLast:" #name, theFrameInfo.getTimeSince(statuses[name].lastSeen));

void FieldFeatureOverview::draw() const
{
  if(Blackboard::getInstance().exists("FrameInfo"))
  {
    const FrameInfo& theFrameInfo = static_cast<const FrameInfo&>(Blackboard::getInstance()["FrameInfo"]);

    PLOT_SINGE_TSL(penaltyArea);
    PLOT_SINGE_TSL(midCircle);
    PLOT_SINGE_TSL(penaltyMarkWithPenaltyAreaLine);

    PLOT("representation:FieldFeatureOverview:timeSinceLast", theFrameInfo.getTimeSince(combinedStatus.lastSeen));
  }
}

void FieldFeatureOverview::operator>>(NaovaMessage& m) const
{
}

bool FieldFeatureOverview::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>&)
{
  ASSERT(m.getMessageID() == id());

  combinedStatus.isValid = false;
  combinedStatus.lastSeen = 0;

  FOREACH_ENUM(Feature, i)
  {
    FieldFeatureStatus& status = statuses[i];
    int8_t container;

    m.bin >> container;
    status.rotation = Angle(static_cast<float>(container) * 180_deg / 127.f);

    m.bin >> container;
    status.translation.x() = static_cast<float>(static_cast<int>(container) << 6);
    m.bin >> container;
    status.translation.y() = static_cast<float>(static_cast<int>(container) << 6);

    m.bin >> container;
    const FrameInfo* theFrameInfo = nullptr;
    if(Blackboard::getInstance().exists("FrameInfo"))
      theFrameInfo = static_cast<const FrameInfo*>(&(Blackboard::getInstance()["FrameInfo"]));
    if(theFrameInfo)
    {
      //this not 100% correct, but it is just for human reading
      status.lastSeen = theFrameInfo->time - (static_cast<unsigned>(container) << 3);

      if((status.isValid = theFrameInfo->getTimeSince(status.lastSeen) < 300))
        combinedStatus.isValid = true;

      combinedStatus.lastSeen = std::max(combinedStatus.lastSeen, status.lastSeen);
    }
  }

  return true;
}
