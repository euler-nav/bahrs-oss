/// @file MagneticHeadingMonitorApi.h
/// @brief Declaration of the API class for magnetic heading output monitor.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef MAGNETIC_HEADING_MONITOR_API_H
#define MAGNETIC_HEADING_MONITOR_API_H

#include "CommonMonitorTypes.h"

namespace NMagneticHeadingMonitorApi
{
  DECLARE_LABELED_ARRAY_TEMPLATE(SArrayLabeledByMagneticHeadingSignal, eMagneticHeading)

  using CRedundantInputData = NMonitorTypes::TRedundantSensorData<NFusionLibCommon::SMagneticHeadingData,
                                                                  NFusionLibCommon::ESensorId::eMagneticHeadingFilter1,
                                                                  NFusionLibCommon::ESensorId::eMagneticHeadingFilter2,
                                                                  NFusionLibCommon::ESensorId::eMagneticHeadingFilter3>;

  using COutputData = NMonitorTypes::TMultidimensionalSignal<SArrayLabeledByMagneticHeadingSignal>;

  namespace NBahrsV2
  {
    /// @brief Get monitor state.
    /// @return Monitor state.
    NMonitorTypes::EMonitorState MagneticHeadingMonitorGetState();

    /// @brief Run redundancy-based magnetic heading signal check.
    /// @param korMeasurements Input redundant measurements.
    /// @return Output magnetic heading data with integrity information.
    COutputData MagneticHeadingMonitorRun(const CRedundantInputData& korMeasurements);

  #ifdef _MSC_VER
    /// @brief Write debug information to a CSV file.
    void MagneticHeadingMonitorWriteDebugOutput();
  #endif // _MSC_VER
  } // namespace NBahrsV2

  namespace NBahrsV3
  {
    /// @brief Get monitor state.
    /// @return Monitor state.
    NMonitorTypes::EMonitorState MagneticHeadingMonitorGetState();

    /// @brief Run redundancy-based magnetic heading signal check.
    /// @param korMeasurements Input redundant measurements.
    /// @return Output magnetic heading data with integrity information.
    COutputData MagneticHeadingMonitorRun(const CRedundantInputData& korMeasurements);

  #ifdef _MSC_VER
    /// @brief Write debug information to a CSV file.
    void MagneticHeadingMonitorWriteDebugOutput();
  #endif // _MSC_VER
  } // namespace NBahrsV3

} // namespace NMagneticHeadingMonitorApi

#endif // MAGNETIC_HEADING_MONITOR_API_H
