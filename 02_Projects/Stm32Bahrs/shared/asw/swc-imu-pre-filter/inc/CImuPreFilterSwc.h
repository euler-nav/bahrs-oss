/// @file CImuPreFilterSwc.h
/// @brief Declaration of the IMU pre-filter SWC class.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_IMU_PRE_FILTER_SWC_H
#define C_IMU_PRE_FILTER_SWC_H

#include <stdint.h>

#include "General/CSoftwareComponentBase.h"
#include "CButterworthLowPassFilter2.h"

/// @brief Software component that applies a 2nd order Butterworth low-pass filter to raw IMU measurements.
/// Currently, only the specific force signals are filtered.
class CImuPreFilterSwc : public CSoftwareComponent<CImuPreFilterSwc, 3U>
{
  friend class CSoftwareComponent<CImuPreFilterSwc, 3U>;
  FORBID_CLASS_COPY_AND_MOVE(CImuPreFilterSwc)
  DECLARE_MANDATORY_APIS(CImuPreFilterSwc)

public:
  /// @brief IMU sensor identifier for pre-filter instances.
  enum class ESensorId : uint8_t
  {
    eImu1,
    eImu2,
    eImu3
  };

  /// @brief Apply the Butterworth low-pass filter to the raw IMU measurement and write the result.
  void Run();

private:

  struct SFilterConfiguration
  {
    ESensorId eSensorId { ESensorId::eImu1 };
    float fCutoffFrequencyHz{ 0.0F };
  };

  explicit CImuPreFilterSwc(SFilterConfiguration oFilterConfig);
  ~CImuPreFilterSwc() = default;

  static constexpr float skfSamplingFrequencyHz_{ 200.0F }; ///< IMU sampling frequency, [Hz]
  static constexpr uint8_t skuMaxInvalidInputCount_{ 5U }; ///< Maximum allowed number of consecutive invalid input datasets before the filter is reset.

  bool readImuDataPort(struct SImuMeasurement& orInput) const;

  void writeFilteredImuDataPort(const struct SImuMeasurement& orOutput) const;

  void resetFilters();

  template<ESensorId eSensorId> static constexpr SFilterConfiguration getFilterConfiguration();

  const ESensorId keSensorId_;
  const float kfCutoffFrequencyHz_; ///< Cutoff frequency of the Butterworth low-pass filter, [Hz]

  uint8_t uInvalidInputCounter_{ 0U };
  bool bIsInitialized_{ false };

  CButterworthLowPassFilter2 oFilterSpecificForceX_{};
  CButterworthLowPassFilter2 oFilterSpecificForceY_{};
  CButterworthLowPassFilter2 oFilterSpecificForceZ_{};
};


#ifdef BAHRS_HW_V3

template<>
constexpr CImuPreFilterSwc::SFilterConfiguration CImuPreFilterSwc::getFilterConfiguration<CImuPreFilterSwc::ESensorId::eImu1>()
{
  return { ESensorId::eImu1, 10.0F };
}

template<>
constexpr CImuPreFilterSwc::SFilterConfiguration CImuPreFilterSwc::getFilterConfiguration<CImuPreFilterSwc::ESensorId::eImu2>()
{
  return { ESensorId::eImu2, 10.0F };
}

template<>
constexpr CImuPreFilterSwc::SFilterConfiguration CImuPreFilterSwc::getFilterConfiguration<CImuPreFilterSwc::ESensorId::eImu3>()
{
  return { ESensorId::eImu3, 10.0F };
}

#else

template<>
constexpr CImuPreFilterSwc::SFilterConfiguration CImuPreFilterSwc::getFilterConfiguration<CImuPreFilterSwc::ESensorId::eImu1>()
{
  return { ESensorId::eImu1, 10.0F };
}

template<>
constexpr CImuPreFilterSwc::SFilterConfiguration CImuPreFilterSwc::getFilterConfiguration<CImuPreFilterSwc::ESensorId::eImu2>()
{
  return { ESensorId::eImu2, 3.0F };
}

template<>
constexpr CImuPreFilterSwc::SFilterConfiguration CImuPreFilterSwc::getFilterConfiguration<CImuPreFilterSwc::ESensorId::eImu3>()
{
  return { ESensorId::eImu3, 3.0F };
}

#endif // BAHRS_HW_V3

#endif // C_IMU_PRE_FILTER_SWC_H
