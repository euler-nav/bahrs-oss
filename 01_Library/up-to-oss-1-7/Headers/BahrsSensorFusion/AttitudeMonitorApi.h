/**
* @file AttitudeMonitorApi.h
* @brief Declaration of the API class for attitude output monitor.
* @author Fedor Baklanov
* @date 16 July 2024
* @copyright Copyright 2024 AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef ATTITUDE_MONITOR_API_H
#define ATTITUDE_MONITOR_API_H

#include "CommonMonitorTypes.h"

namespace NAttitudeMonitorApi
{
  DECLARE_LABELED_ARRAY_TEMPLATE(SArrayLabeledByAttitudeSignals, eRoll, ePitch)

  using CRedundantInputData = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SAttitudeOutputData,
                                                                  NFusionLibCommon::ESensorId::eBahrsFilter1,
                                                                  NFusionLibCommon::ESensorId::eBahrsFilter2,
                                                                  NFusionLibCommon::ESensorId::eBahrsFilter3>;

  using COutputData = NMonitorTypes::TMultidimensionalSignal<SArrayLabeledByAttitudeSignals>;

  /**
   * @brief Get monitor state.
   * @return Monitor state
  */
  NMonitorTypes::EMonitorState AttitudeMonitorGetState();

  /**
   * @brief Run redundancy-based attitude signal check.
   * @param korMeasurements Input redundant measurements.
   * @return Output attitude data with integrity information.
  */
  COutputData AttitudeMonitorRun(const CRedundantInputData& korMeasurements);

#ifdef _MSC_VER
  /**
   * @brief Write debug information to a CSV file.
  */
  void AttitudeMonitorWriteDebugOutput();
#endif /* _MSC_VER */
}

#endif /* ATTITUDE_MONITOR_API_H */
