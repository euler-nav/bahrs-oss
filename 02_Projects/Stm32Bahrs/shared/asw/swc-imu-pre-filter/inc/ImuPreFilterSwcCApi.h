/// @file ImuPreFilterSwcCApi.h
/// @brief Declaration of C APIs of the IMU pre-filter SWC.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef IMU_PRE_FILTER_SWC_C_API_H
#define IMU_PRE_FILTER_SWC_C_API_H

#ifdef __cplusplus
  #error This header must not be included in .cpp files.
#endif

#include <stdint.h>

/// @brief A C-wrapper of CImuPreFilterSwc::Run().
/// @param uSensorIndex Zero-based IMU sensor index.
void ImuPreFilterSwcRun(uint32_t uSensorIndex);

#endif // IMU_PRE_FILTER_SWC_C_API_H
