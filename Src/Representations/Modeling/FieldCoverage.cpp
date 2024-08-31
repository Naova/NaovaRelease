/**
 * @file FieldCoverage.cpp
 *
 * Implementation to send information about the field coverage.
 *
 * @author Nicole Schrader
 */

#include "FieldCoverage.h"

/**
 * Encodes the time difference into two bits.
 *
 * @param diff the time difference
 * @return two bits representing the time difference
 */
int encodeTimeDifference(unsigned diff);

/**
 * Decodes two bits into the time difference.
 *
 * @param code two bits representing the time difference
 * @return the time difference
 */
unsigned decodeTimeDifference(int code);

void FieldCoverage::operator>>(NaovaMessage& m) const
{
}

void FieldCoverage::operator<<(const NaovaMessage&)
{
  for(size_t y = 0; y < lines.size(); y++)
  {
    lines[y].timestamps.clear();
  }
}

bool FieldCoverage::handleArbitraryMessage(InMessage& m, const std::function<unsigned(unsigned)>& toLocalTimestamp)
{
  ASSERT(m.getMessageID() == id());

  unsigned time = 0;
  m.bin >> time;

  int y = 0;
  int cnt = 0;
  uint8_t cov;
  while(m.getBytesLeft())
  {
    lines[y].y = y;

    for(int x = 0; x < 18; x++)
    {
      if(cnt % 8 == 0 && m.getBytesLeft())
        m.bin >> cov;

      lines[y].timestamps.emplace_back(std::max<int>(0, toLocalTimestamp(time) - decodeTimeDifference(cov >> 6)));
      cov <<= 2;
      cnt += 2;
    }
    y++;
  }
  return true;
}

unsigned decodeTimeDifference(int code)
{
  if(code == 3)
    return 60000;
  if(code == 2)
    return 20000;
  if(code == 1)
    return 5000;
  return 1000;
}

int encodeTimeDifference(unsigned diff)
{
  if(diff > 20000)
    return 3;
  if(diff > 5000)
    return 2;
  if(diff > 1000)
    return 1;
  return 0;
}
