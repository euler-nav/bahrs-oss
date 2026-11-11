/// @file BahrsFilterSwcCApi.h
/// @brief Declaration C APIs of the BAHRS filter SWC.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef BAHRS_FILTER_SWC_C_API_H
#define BAHRS_FILTER_SWC_C_API_H

#ifdef __cplusplus
  #error This header must not be included in .cpp files.
#endif

#include <stdint.h>

/// @brief A C-wrapper of CBahrsFilterSwc::SetImuInput().
/// @param uFilterIndex Index of the filter SWC instance.
void BahrsFilterSwcSetImuInput(uint32_t uFilterIndex);

#endif // BAHRS_FILTER_SWC_C_API_H

