/**
* @file ImuMonitorApi.h
* @brief Declaration of the IMU monitor API.
* @author Fedor Baklanov
* @date 6 December 2023
* @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef IMU_MONITOR_API_H
#define IMU_MONITOR_API_H

#include "CommonMonitorTypes.h"

namespace NImuMonitorApi
{
  DECLARE_LABELED_ARRAY_TEMPLATE(SArrayLabeledByImuSignals, eSpecificForceX, eSpecificForceY, eSpecificForceZ, eAngularRateX, eAngularRateY, eAngularRateZ)

  using CRedundantInputData = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SImuMeasurement,
                                                                  NFusionLibCommon::ESensorId::eScha63T,
                                                                  NFusionLibCommon::ESensorId::eIcm20789Imu1,
                                                                  NFusionLibCommon::ESensorId::eIcm20789Imu2>;

  using COutputData = NMonitorTypes::TMultidimensionalSignal<SArrayLabeledByImuSignals>;

  /**
   * @brief Get monitor state.
   * @return Monitor state
  */
  NMonitorTypes::EMonitorState ImuMonitorGetState();

  /**
   * @brief Run redundancy-base IMU signal check.
   * @param korMeasurements Input redundant measurements.
   * @return Output IMU measurement.
  */
  COutputData ImuMonitorRun(const CRedundantInputData& korMeasurements);

#ifdef _MSC_VER
  /**
   * @brief Write debug information to a CSV file.
  */
  void ImuMonitorWriteDebugOutput();
#endif /* _MSC_VER */
}

#endif /* IMU_MONITOR_API_H */
