/**
 * @file Scha63TDriverTypes.h
 * @brief Type declarations for the SCHA63T driver.
 * @author Fedor Baklanov
 * @date 01 June 2022
 */

#ifndef SCHA63T_DRIVER_TYPES_H
#define SCHA63T_DRIVER_TYPES_H

#ifndef __cplusplus
#error This file must not be included in C code.
#endif /* __cplusplus */

#include <array>

#define SAMPLE_COUNT (15U) ///< With 3.01 kHz sample rate, this produces 200 Hz Output Data Rate (ODR)

struct SScha63TMeasurement
{
  int16_t iSpecificForceX_{ 0 };
  int16_t iSpecificForceY_{ 0 };
  int16_t iSpecificForceZ_{ 0 };
  int16_t iAngularRateX_{ 0 };
  int16_t iAngularRateY_{ 0 };
  int16_t iAngularRateZ_{ 0 };
  int16_t iTemperatureUno_{ 0 };
  int16_t iTemperatureDue_{ 0 };
  uint8_t uValid_{ 0U };
};

struct SScha63TDataset
{
  std::array<SScha63TMeasurement, SAMPLE_COUNT> oMeasurements_;
  uint64_t uTimestampFirstUs_{ 0U };
  uint64_t uTimestampLastUs_{ 0U };
};

struct SScha63TStatusUno
{
  uint16_t uSummaryStatus_{ 0U };
  uint16_t uRateStatus_{ 0U };
  uint16_t uAccelerometerStatus_{ 0U };
  uint16_t uCommonStatus1_{ 0U };
  uint16_t uCommonStatus2_{ 0U };
};


struct SScha63TStatusDue
{
  uint16_t uSummaryStatus_{ 0U };
  uint16_t uRateStatus1_{ 0U };
  uint16_t uRateStatus2_{ 0U };
  uint16_t uCommonStatus1_{ 0U };
  uint16_t uCommonStatus2_{ 0U };
};


#endif /* SCHA63T_DRIVER_TYPES_H */
