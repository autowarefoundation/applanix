#pragma once

#include <stdint.h>

#include <applanix_msgs/msg/time_distance_group.hpp>

namespace applanix_msgs
{

/**
 * Takes a 4 bit value and converts to applanix_msgs::TimeSource
 */
static uint8_t bitsToTimeSource(const uint8_t bits)
{
  switch (bits)
  {
    case 0:
      return msg::TimeDistanceGroup::TIME_SOURCE_POS;
    case 1:
      return msg::TimeDistanceGroup::TIME_SOURCE_GPS;
    case 2:
      return msg::TimeDistanceGroup::TIME_SOURCE_UTC;
    case 3:
      return msg::TimeDistanceGroup::TIME_SOURCE_USER;
    default:
      return 255;  // invalid value
  }
}

static uint8_t getTimeType1(const uint8_t flag)
{
  return bitsToTimeSource(flag & 0b0000'1111);
}

static uint8_t getTimeType2(const uint8_t flag)
{
  return bitsToTimeSource((flag & 0b1111'0000) >> 4);
}

}  // namespace applanix_msgs
