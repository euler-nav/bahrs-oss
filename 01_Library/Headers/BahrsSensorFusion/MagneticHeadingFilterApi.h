/**
* @file MagneticHeadingFilterApi.h
* @brief Declaration of the magnetic heading filter API.
* @author Fedor Baklanov
* @date 6 December 2023
* @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef MAGNETIC_HEADING_FILTER_API_H
#define MAGNETIC_HEADING_FILTER_API_H

#include <stdint.h>

namespace NMagneticHeadingFilterApi
{
  struct SMagnetometerMeasurement
  {
    float fVectorX_{ 0.0F }; ///< Magnetic field vector in Gauss axis X
    float fVectorY_{ 0.0F }; ///< Magnetic field vector in Gauss axis Y
    float fVectorZ_{ 0.0F }; ///< Magnetic field vector in Gauss axis Z
    uint64_t uTimestampUs_{ 0U }; ///< Timestamp in microseconds
  };

  struct SOutputData
  {
    float fMagneticHeading_{ 0.0F }; ///< Estimated magnetic heading, [rad], from 0 to 2 pi
    uint64_t uTimestampUs_{ 0U }; ///< Timestamp, [us]
    bool bValid_{ false }; ///< True -- valid, false otherwise
  };

  /**
   * @brief Pass new attitude parameters to the filter.
   * Provided inputs must be valid. Validity checks shall be performed by the caller.
   * @param uAttitudeTimeUs Timestamp of attitude data, [us]
   * @param fRoll Roll angle, [rad], from -pi to pi, including +-pi.
   * @param fPitch Pitch angle, [rad], from -pi/2 to pi/2, NOT including +-pi/2.
  */
  void MagneticHeadingFilterSetInput(uint64_t uAttitudeTimeUs, float fRoll, float fPitch);

  /**
   * @brief Makes a snapshot of inputs.
   * Shall be called in a critical before CMagneticHeadingFilter::Step().
  */
  void MagneticHeadingFilterPrepareInputs();

  /**
   * @brief The routine implements estimation of magnetic heading
   * @param uCurrentTimeUs_ Current time in microseconds
   * @param orMagnetometerData Magnetometer measurement
  */
  void MagneticHeadingFilterStep(uint64_t uCurrentTimeUs_, const SMagnetometerMeasurement& orMagnetometerData);

  /**
   * @brief Retrieve output data.
   * @return Output data.
  */
  SOutputData MagneticHeadingFilterGetOutput();

  /**
   * @brief Reset the filter to initial state.
  */
  void MagneticHeadingFilterReset();
}

#endif /* MAGNETIC_HEADING_FILTER_API_H */
