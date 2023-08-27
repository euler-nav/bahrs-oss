#ifndef BAHRS_FILTER_API_H
#define BAHRS_FILTER_API_H

#include "CQuaternion.h"
#include "ClosedLoopErrorStateKfApi.h"

namespace NBahrsFilterApi
{
  /**
   * @brief Input IMU data struct.
  */
  struct SImuData
  {
    uint64_t uTimestampUs_{ 0U };                        ///< Timestamp in microseconds.
    Eigen::Vector3f oSpecificForce_{ 0.0F, 0.0F, 0.0F }; ///< Measured specific force.
    Eigen::Vector3f oAngularRate_{ 0.0F, 0.0F, 0.0F };   ///< Measured angular rate.
  };

  /**
   * @brief Input barometer measurements.
  */
  struct SPressureData
  {
    uint64_t uTimestampUs_{ 0U };      ///< Timestamp in microseconds.
    float fPressureInPascals{ 0.0F }; ///< Measured pressure in pascals.
  };

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
  */
  void BahrsFilterSetInput(const SImuData& korImuData);

  /**
   * @brief Pass input to the filter.
   * The function may need to be called within a critical section.
   * @param korPressureData Reference to pressure data struct.
  */
  void BahrsFilterSetInput(const SPressureData& korPressureData);

  /**
   * @brief Process inputs or make a snaphot of buffers before Step()
   * The function may need to be called within a critical section.
  */
  void BahrsFilterPrepareInputs();

  /**
   * @brief Do estimation.
   * @param uTimeUs Time of the call.
  */
  void BahrsFilterStep(uint64_t uTimeUs);

  /**
   * @brief Clean-up at the end of an epoch.
   * The function may need to be called within a critical section.
  */
  void BahrsFilterCompleteEpoch();

  /**
   * @brief Retrieve output information.
   * @return Filter output data.
  */
  SOutputData BahrsFilterGetOutput();
}

#endif /* BAHRS_FILTER_API_H */
