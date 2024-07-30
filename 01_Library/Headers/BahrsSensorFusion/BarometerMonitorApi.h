/**
* @file BarometerMonitorApi.h
* @brief Declaration of the barometer monitor API.
* @author Fedor Baklanov
* @date 21 February 2024
* @copyright Copyright 2024 AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef BAROMETER_MONITOR_API_H
#define BAROMETER_MONITOR_API_H

#include "CommonMonitorTypes.h"

namespace NBaroMonitorApi
{
  DECLARE_LABELED_ARRAY_TEMPLATE(SArrayLabeledByPressureSignal, ePressure)

  using CRedundantInputData = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SBarometerData,
                                                                  NFusionLibCommon::ESensorId::eBmp384,
                                                                  NFusionLibCommon::ESensorId::eIcm20789Baro1,
                                                                  NFusionLibCommon::ESensorId::eIcm20789Baro2>;

  using COutputData = NMonitorTypes::TMultidimensionalSignal<SArrayLabeledByPressureSignal>;

  /**
   * @brief Get monitor state.
   * @return Monitor state
  */
  NMonitorTypes::EMonitorState BarometerMonitorGetState();

  /**
   * @brief Run redundancy-based pressure signal check.
   * @param korMeasurements Input redundant measurements.
   * @return Output pressure data with integrity information.
  */
  COutputData BarometerMonitorRun(const CRedundantInputData& korMeasurements);

#ifdef _MSC_VER
  /**
   * @brief Write debug information to a CSV file.
  */
  void BarometerMonitorWriteDebugOutput();
#endif /* _MSC_VER */
}

#endif /* BAROMETER_MONITOR_API_H */
