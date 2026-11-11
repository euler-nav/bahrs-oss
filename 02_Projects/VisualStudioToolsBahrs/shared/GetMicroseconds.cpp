/// @file GetMicroseconds.cpp
/// @brief Implementation of replay-time GetMicroseconds for PC tools.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "GetMicroseconds.h"

uint64_t GetMicroseconds()
{
  extern uint64_t uReplayTimestampUs;
  return uReplayTimestampUs;
}
