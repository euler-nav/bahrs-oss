/// @file ImuMonitorApi.h
/// @brief Declaration of the IMU monitor API.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef IMU_MONITOR_API_H
#define IMU_MONITOR_API_H

#include "CommonMonitorTypes.h"

namespace NImuMonitorApi
{

DECLARE_LABELED_ARRAY_TEMPLATE(SArrayLabeledByImuSignals, eSpecificForceX, eSpecificForceY, eSpecificForceZ, eAngularRateX, eAngularRateY, eAngularRateZ)

using CRedundantInputDataBahrsV2 = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SImuMeasurement,
                                                                       NFusionLibCommon::ESensorId::eScha63T,
                                                                       NFusionLibCommon::ESensorId::eIcm20789Imu1,
                                                                       NFusionLibCommon::ESensorId::eIcm20789Imu2>;

using CRedundantInputDataBahrsV3 = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SImuMeasurement,
                                                                       NFusionLibCommon::ESensorId::eScha63T,
                                                                       NFusionLibCommon::ESensorId::eBmi270,
                                                                       NFusionLibCommon::ESensorId::eAsm330>;

using COutputData = NMonitorTypes::TMultidimensionalSignal<SArrayLabeledByImuSignals>;


/// @brief Run redundancy-based IMU signal check.
/// @param korMeasurements Input redundant measurements.
/// @return Output IMU measurement.
COutputData ImuMonitorRun(const CRedundantInputDataBahrsV2& korMeasurements);

/// @brief Run redundancy-based IMU signal check.
/// @param korMeasurements Input redundant measurements.
/// @return Output IMU measurement.
COutputData ImuMonitorRun(const CRedundantInputDataBahrsV3& korMeasurements);

#ifdef _MSC_VER

/// @brief Write debug information to a CSV file.
void ImuMonitorWriteDebugOutputBahrsV2();

/// @brief Write debug information to a CSV file.
void ImuMonitorWriteDebugOutputBahrsV3();

#endif // _MSC_VER

} // namespace NImuMonitorApi

#endif // IMU_MONITOR_API_H
