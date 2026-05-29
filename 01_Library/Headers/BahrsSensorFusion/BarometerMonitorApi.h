/// @file BarometerMonitorApi.h
/// @brief Declaration of the barometer monitor API.
/// @copyright Copyright 2024 AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef BAROMETER_MONITOR_API_H
#define BAROMETER_MONITOR_API_H

#include "CommonMonitorTypes.h"

namespace NBaroMonitorApi
{
  DECLARE_LABELED_ARRAY_TEMPLATE(SArrayLabeledByPressureSignal, ePressure)

  using CRedundantInputDataBahrsV2 = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SBarometerData,
                                                                         NFusionLibCommon::ESensorId::eBmp384,
                                                                         NFusionLibCommon::ESensorId::eIcm20789Baro1,
                                                                         NFusionLibCommon::ESensorId::eIcm20789Baro2>;

  using CRedundantInputDataBahrsV3 = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SBarometerData,
                                                                         NFusionLibCommon::ESensorId::eIcp20100,
                                                                         NFusionLibCommon::ESensorId::eBmp384,
                                                                         NFusionLibCommon::ESensorId::eLps22>;

  using COutputData = NMonitorTypes::TMultidimensionalSignal<SArrayLabeledByPressureSignal>;

  COutputData BarometerMonitorRun(const CRedundantInputDataBahrsV2& korMeasurements);
  COutputData BarometerMonitorRun(const CRedundantInputDataBahrsV3& korMeasurements);

#ifdef _MSC_VER
  void BarometerMonitorWriteDebugOutputBahrsV2();
  void BarometerMonitorWriteDebugOutputBahrsV3();
#endif // _MSC_VER
}

#endif // BAROMETER_MONITOR_API_H
