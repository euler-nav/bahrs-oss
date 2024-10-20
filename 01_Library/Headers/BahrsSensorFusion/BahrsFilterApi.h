/**
* @file BahrsFilterApi.h
* @brief Declaration of the BAHRS filter API.
* @author Fedor Baklanov
* @date 6 December 2023
* @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef BAHRS_FILTER_API_H
#define BAHRS_FILTER_API_H

#include "CQuaternion.h"
#include "ClosedLoopErrorStateKfApi.h"
#include "MeasurementStructs.h"

namespace NBahrsFilterApi
{
  /**
   * @brief BAHRS state vector struct.
  */
  struct SBahrsState
  {
    float fHeight_{ 0.0F };
    float fVelocityDown_{ 0.0F };
    CQuaternion oQuaternionBodyToNed_;
    Eigen::Vector3f oAccelerometerBias_{ 0.0F, 0.0F, 0.0F };
    Eigen::Vector3f oGyroscopeBias_{ 0.0F, 0.0F, 0.0F };

    SBahrsState& operator+=(const SBahrsState& korRight);
    SBahrsState operator+(const SBahrsState& korRight);
    SBahrsState operator*(float fScalar) const;
  };

  /**
   * @brief A struct to store standard deviations of the BAHRS state vector.
  */
  struct SBahrsStateStd
  {
    float fHeight_{ 0.0F };
    float fVelocityDown_{ 0.0F };
    Eigen::Vector3f oAttitude_{ 0.0F, 0.0F, 0.0F };
    Eigen::Vector3f oAccelerometerBias_{ 0.0F, 0.0F, 0.0F };
    Eigen::Vector3f oGyroscopeBias_{ 0.0F, 0.0F, 0.0F };
  };

  using EFilterModes = CClosedLoopErrorStateKfApi::EFilterModes;

  /**
   * @brief Filter output struct.
  */
  struct SOutputData
  {
    SBahrsState oState_;                              ///< State estimate.
    SBahrsStateStd oStateStd_;                        ///< Estimated standard deviation of the state estimate.
    uint64_t uTimestampUs_{ 0U };                    ///< Timestamp in microseconds.
    EFilterModes eFilterMode_{ EFilterModes::IDLE }; ///< Current filter mode.
  };

  /**
   * @brief Pass input to the filter.
   * The function may need to be called within a critical section.
   * @param korImuData Reference to IMU data struct.
   * @param uFilterIndex Filter instance to call.
  */
  void BahrsFilterSetInput(const NFusionLibCommon::SImuMeasurement& korImuData, uint32_t uFilterIndex);

  /**
   * @brief Pass input to the filter.
   * The function may need to be called within a critical section.
   * @param korPressureData Reference to pressure data struct.
   * @param uFilterIndex Filter instance to call.
  */
  void BahrsFilterSetInput(const NFusionLibCommon::SBarometerData& korPressureData, uint32_t uFilterIndex);

  /**
   * @brief Process inputs or make a snaphot of buffers before Step()
   * The function may need to be called within a critical section.
   * @param uFilterIndex Filter instance to call.
  */
  void BahrsFilterPrepareInputs(uint32_t uFilterIndex);

  /**
   * @brief Do estimation.
   * @param uTimeUs Time of the call.
   * @param uFilterIndex Filter instance to call.
  */
  void BahrsFilterStep(uint64_t uTimeUs, uint32_t uFilterIndex);

  /**
   * @brief Clean-up at the end of an epoch.
   * The function may need to be called within a critical section.
   * @param uFilterIndex Filter instance to call.
  */
  void BahrsFilterCompleteEpoch(uint32_t uFilterIndex);

  /**
   * @brief Retrieve output information.
   * @param uFilterIndex Filter instance to call.
   * @return Filter output data.
  */
  SOutputData BahrsFilterGetOutput(uint32_t uFilterIndex);

  /**
   * @brief Get number of available filter instances.
   * @return Number of filter instances.
   */
  uint32_t GetBahrsFilterCount();
}

#endif /* BAHRS_FILTER_API_H */
