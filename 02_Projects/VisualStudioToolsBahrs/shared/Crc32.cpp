/// @file Crc32.cpp
/// @brief Implementation of CRC-32 (MPEG-2) for 32-bit word arrays.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "Crc32.h"

uint32_t Crc32(const uint32_t* upData, size_t uLength)
{
  uint32_t uCrc = 0xFFFFFFFFU;

  for (size_t uPos = 0U; uPos < uLength; ++uPos)
  {
    uCrc = uCrc ^ upData[uPos];

    for (uint8_t uBit = 0U; uBit < 32U; ++uBit)
    {
      if (uCrc & 0x80000000U)
      {
        uCrc = (uCrc << 1U) ^ 0x04C11DB7U;
      }
      else
      {
        uCrc = (uCrc << 1U);
      }
    }
  }

  return uCrc;
}
