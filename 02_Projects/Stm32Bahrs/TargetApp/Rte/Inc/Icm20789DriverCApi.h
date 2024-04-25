/**
 * @file Icm20789DriverCApi.h
 * @brief Declaration of the ICM20789 Driver C APIs
 * @author Fedor Baklanov
 * @date 3 December 2023
 * @copyright Copyright 2023 AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef ICM_20789_DRIVER_C_API_H
#define ICM_20789_DRIVER_C_API_H

#ifdef __cplusplus
#error This file shall not be included in .cpp files
#endif

/**
 * \brief Initiate data transmission from the IMU in DMA mode.
 * \param uInstanceIndex Index of the sensor driver instance.
 * \return 0 -- success, 1 -- failure.
 */
int Icm20789ImuDataDmaRequest(unsigned uInstanceIndex);

/**
 * \brief Parse IMU data received by the DMA controller.
 * \param uInstanceIndex
 */
void Icm20789ParseImuDataDma(unsigned uInstanceIndex);

/**
 * \brief Initiate data transmission from the pressure sensor in DMA mode.
 * \param uInstanceIndex Index of the sensor driver instance.
 * \return 0 -- success, 1 -- failure.
 */
int Icm20789PressureDataDmaRequest(unsigned uInstanceIndex);

/**
 * \brief Trigger pressure measuring process.
 * \param uInstanceIndex Index of the sensor driver instance.
 * \return 0 -- success, 1 -- failure.
 */
int Icm20789TriggerPressureMeasurement(unsigned uInstanceIndex);

/**
 * \brief Parse pressure data received by the DMA controller.
 * \param uInstanceIndex
 */
void Icm20789ParsePressureDataDma(unsigned uInstanceIndex);

/**
 * Invalidate data in the IMU data port of the software component.
 */
void Icm20789InvalidateImuOutputPort(unsigned uInstanceIndex);

/**
 * Invalidate data in the pressure data port of the software component.
 */
void Icm20789InvalidatePressureOutputPort(unsigned uInstanceIndex);

#endif /* ICM_20789_DRIVER_C_API_H */

