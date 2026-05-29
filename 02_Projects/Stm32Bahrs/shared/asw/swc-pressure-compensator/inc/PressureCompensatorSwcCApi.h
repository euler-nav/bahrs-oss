/// @file PressureCompensatorSwcCApi.h
/// @brief Declaration of C APIs of the pressure compensator SWC.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef PRESSURE_COMPENSATOR_SWC_C_API_H
#define PRESSURE_COMPENSATOR_SWC_C_API_H

#ifdef __cplusplus
  #error This header must not be included in .cpp files.
#endif

#include <stdint.h>

/// @brief A C-wrapper of CPressureCompensator::CompensateMeasurements().
/// @param uSensorIndex Zero-based pressure sensor index.
void PressureCompensatorSwcCompensate(uint32_t uSensorIndex);

#endif // PRESSURE_COMPENSATOR_SWC_C_API_H
