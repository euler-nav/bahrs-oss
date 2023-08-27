/**
 * @file UintToBool.h
 * @brief Conversion of integer validity flags to boolean.
 * @author Fedor Baklanov
 * @date 23 June 2022
 */

#ifndef UINT_TO_BOOL_H
#define UINT_TO_BOOL_H

#include <stdint.h>

/**
 * Convert integer validity flag to boolean.
 * \param uValid Unsigned validity flag.
 * \return True is uValid is greater than zero, false is uValid is equal to zero.
 */
inline bool UintToBool(uint8_t uValid)
{
  return (uValid == 0U) ? false : true;
}

/**
 * Convert boolean validity flag to integer validity.
 * \param bValid Boolean validity flag.
 * \return Unsigned validity flag.
 */
inline uint8_t BoolToUint(bool bValid)
{
  return (true == bValid) ? 1U : 0U;
}

#endif /* UINT_TO_BOOL_H */
