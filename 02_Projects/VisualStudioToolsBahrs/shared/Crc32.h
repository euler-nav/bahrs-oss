/// @file Crc32.h
/// @brief Declaration of CRC-32 (MPEG-2) helper for 32-bit word arrays.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_CRC32_H
#define C_CRC32_H


#include <cstddef>
#include <cstdint>

/// @brief Calculate CRC-32 (MPEG-2 polynomial) for 32-bit word arrays.
/// @param upData Pointer to an array of 32-bit words.
/// @param uLength Number of 32-bit words.
/// @return CRC value.
uint32_t Crc32(const uint32_t* upData, size_t uLength);

#endif // C_CRC32_H
