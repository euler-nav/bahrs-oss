/**
* @file VerticalChannelMonitorApi.h
* @brief Declaration of the API class for a vertical channel (height and velocity down) monitor.
* @author Fedor Baklanov
* @date 17 July 2024
* @copyright Copyright 2024 AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef VERTICAL_CHANNEL_MONITOR_API_H
#define VERTICAL_CHANNEL_MONITOR_API_H

#include "CommonMonitorTypes.h"

namespace NVerticalChannelMonitorApi
{
  DECLARE_LABELED_ARRAY_TEMPLATE(SArrayLabeledByVerticalChannelSignals, eHeight, eVelocityDown)

  using CRedundantInputData = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SVerticalChannelData,
                                                                  NFusionLibCommon::ESensorId::eBahrsFilter1,
                                                                  NFusionLibCommon::ESensorId::eBahrsFilter2,
                                                                  NFusionLibCommon::ESensorId::eBahrsFilter3>;

  using COutputData = NMonitorTypes::TMultidimensionalSignal<SArrayLabeledByVerticalChannelSignals>;

  /**
   * @brief Get monitor state.
   * @return Monitor state
  */
  NMonitorTypes::EMonitorState VerticalChannelMonitorGetState();

  /**
   * @brief Run redundancy-based vertical channel signal check.
   * @param korMeasurements Input redundant measurements.
   * @return Output vertical channel data with integrity information.
  */
  COutputData VerticalChannelMonitorRun(const CRedundantInputData& korMeasurements);

#ifdef _MSC_VER
  /**
   * @brief Write debug information to a CSV file.
  */
  void VerticalChannelMonitorWriteDebugOutput();
#endif /* _MSC_VER */
}

#endif /* VERTICAL_CHANNEL_MONITOR_API_H */
