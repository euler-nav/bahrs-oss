/// @file UintToBool.h
/// @brief Declaration of conversion of integer validity flags to boolean.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef UINT_TO_BOOL_H
#define UINT_TO_BOOL_H

#include <stdint.h>

/// @brief Convert integer validity flag to boolean.
/// @param uValid Unsigned validity flag.
/// @return True is uValid is greater than zero, false is uValid is equal to zero.
inline bool UintToBool(uint8_t uValid)
{
  return (uValid == 0U) ? false : true;
}

/// @brief Convert boolean validity flag to integer validity.
/// @param bValid Boolean validity flag.
/// @return Unsigned validity flag.
inline uint8_t BoolToUint(bool bValid)
{
  return (true == bValid) ? 1U : 0U;
}

#endif // UINT_TO_BOOL_H
