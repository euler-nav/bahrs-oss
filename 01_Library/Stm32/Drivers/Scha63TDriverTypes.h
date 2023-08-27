/**
 * @file Scha63TDriverTypes.h
 * @brief Type declarations for the SCHA63T driver.
 * @author Fedor Baklanov
 * @date 01 June 2022
 */

#ifndef SCHA63T_DRIVER_TYPES_H
#define SCHA63T_DRIVER_TYPES_H

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define SAMPLE_COUNT 30U ///< With 6.02 kHz sample rate, this produces 200 Hz Output Data Rate (ODR)

typedef struct SScha63TMeasurement
{
  int16_t iSpecificForceX_;
  int16_t iSpecificForceY_;
  int16_t iSpecificForceZ_;
  int16_t iAngularRateX_;
  int16_t iAngularRateY_;
  int16_t iAngularRateZ_;
  int16_t iTemperatureUno_;
  int16_t iTemperatureDue_;
  uint8_t uValid_;
} SScha63TMeasurement_t;

typedef struct SScha63TDataset
{
  SScha63TMeasurement_t aoMeasurements_[SAMPLE_COUNT];
  uint64_t uTimestampFirstUs_;
  uint64_t uTimestampLastUs_;
} SScha63TDataset_t;

typedef struct SScha63TStatusUno
{
  uint16_t uSummaryStatus_;
  uint16_t uRateStatus_;
  uint16_t uAccelerometerStatus_;
  uint16_t uCommonStatus1_;
  uint16_t uCommonStatus2_;
} SScha63TStatusUno_t;


typedef struct SScha63TStatusDue
{
  uint16_t uSummaryStatus_;
  uint16_t uRateStatus1_;
  uint16_t uRateStatus2_;
  uint16_t uCommonStatus1_;
  uint16_t uCommonStatus2_;
} SScha63TStatusDue_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */


#endif /* SCHA63T_DRIVER_TYPES_H */
