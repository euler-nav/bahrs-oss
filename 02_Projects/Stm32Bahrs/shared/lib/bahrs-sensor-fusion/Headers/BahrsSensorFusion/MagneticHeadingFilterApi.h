/// @file MagneticHeadingFilterApi.h
/// @brief Declaration of the magnetic heading filter API.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef MAGNETIC_HEADING_FILTER_API_H
#define MAGNETIC_HEADING_FILTER_API_H

#include <stdint.h>
#include "MeasurementStructs.h"

namespace NMagneticHeadingFilterApi
{
  struct SOutputData
  {
    float fMagneticHeading_{ 0.0F }; ///< Estimated magnetic heading, [rad], from 0 to 2 pi
    uint64_t uTimestampUs_{ 0U }; ///< Timestamp, [us]
    bool bValid_{ false }; ///< True -- valid, false otherwise
  };

  namespace NBahrsV2
  {
    /// @brief Pass new attitude parameters to the filter.
    /// Provided inputs must be valid. Validity checks shall be performed by the caller.
    /// @param uAttitudeTimeUs Timestamp of attitude data, [us]
    /// @param fRoll Roll angle, [rad], from -pi to pi, including +-pi.
    /// @param fPitch Pitch angle, [rad], from -pi/2 to pi/2, NOT including +-pi/2.
    /// @param uFilterIndex Filter instance to call.
    void MagneticHeadingFilterSetInput(uint64_t uAttitudeTimeUs, float fRoll, float fPitch, uint32_t uFilterIndex);

    /// @brief Makes a snapshot of inputs.
    /// Shall be called in a critical section before MagneticHeadingFilterStep().
    /// @param uFilterIndex Filter instance to call.
    void MagneticHeadingFilterPrepareInputs(uint32_t uFilterIndex);

    /// @brief The routine implements estimation of magnetic heading.
    /// @param uCurrentTimeUs_ Current time in microseconds.
    /// @param orMagnetometerData Magnetometer measurement.
    /// @param uFilterIndex Filter instance to call.
    void MagneticHeadingFilterStep(uint64_t uCurrentTimeUs_, const NFusionLibCommon::SMagnetometerData& orMagnetometerData, uint32_t uFilterIndex);

    /// @brief Retrieve output data.
    /// @param uFilterIndex Filter instance to call.
    /// @return Output data.
    SOutputData MagneticHeadingFilterGetOutput(uint32_t uFilterIndex);

    /// @brief Reset the filter to initial state.
    /// @param uFilterIndex Filter instance to call.
    void MagneticHeadingFilterReset(uint32_t uFilterIndex);

    /// @brief Get number of available filter instances.
    /// @return Number of filter instances.
    uint32_t GetMagneticHeadingFilterCount();
  } // namespace NBahrsV2

  namespace NBahrsV3
  {
    /// @brief Pass new attitude parameters to the filter.
    /// Provided inputs must be valid. Validity checks shall be performed by the caller.
    /// @param uAttitudeTimeUs Timestamp of attitude data, [us]
    /// @param fRoll Roll angle, [rad], from -pi to pi, including +-pi.
    /// @param fPitch Pitch angle, [rad], from -pi/2 to pi/2, NOT including +-pi/2.
    /// @param uFilterIndex Filter instance to call.
    void MagneticHeadingFilterSetInput(uint64_t uAttitudeTimeUs, float fRoll, float fPitch, uint32_t uFilterIndex);

    /// @brief Makes a snapshot of inputs.
    /// Shall be called in a critical section before MagneticHeadingFilterStep().
    /// @param uFilterIndex Filter instance to call.
    void MagneticHeadingFilterPrepareInputs(uint32_t uFilterIndex);

    /// @brief The routine implements estimation of magnetic heading.
    /// @param uCurrentTimeUs_ Current time in microseconds.
    /// @param orMagnetometerData Magnetometer measurement.
    /// @param uFilterIndex Filter instance to call.
    void MagneticHeadingFilterStep(uint64_t uCurrentTimeUs_, const NFusionLibCommon::SMagnetometerData& orMagnetometerData, uint32_t uFilterIndex);

    /// @brief Retrieve output data.
    /// @param uFilterIndex Filter instance to call.
    /// @return Output data.
    SOutputData MagneticHeadingFilterGetOutput(uint32_t uFilterIndex);

    /// @brief Reset the filter to initial state.
    /// @param uFilterIndex Filter instance to call.
    void MagneticHeadingFilterReset(uint32_t uFilterIndex);

    /// @brief Get number of available filter instances.
    /// @return Number of filter instances.
    uint32_t GetMagneticHeadingFilterCount();
  } // namespace NBahrsV3

} // namespace NMagneticHeadingFilterApi

#endif // MAGNETIC_HEADING_FILTER_API_H
