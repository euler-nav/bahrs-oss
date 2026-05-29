/// @file CAsm330lhhDriver.cpp
/// @brief Implementation of the BMI270 driver software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_LPS22HH_DRIVER_H
#define C_LPS22HH_DRIVER_H

#include "General/CSoftwareComponentBase.h"


class CLps22hhDriver : public CSoftwareComponent<CLps22hhDriver, 1U>
{
  friend class CSoftwareComponent<CLps22hhDriver, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CLps22hhDriver)
  DECLARE_MANDATORY_APIS(CLps22hhDriver)

public:
  /// @brief Poll the sensor.
  /// The timestamp is expected to be generated in the data-ready interrupt handler.
  /// @param uTimestampUs Timestamp of the measurement in microseconds.
  void PollSensor(uint64_t uTimestampUs);

private:
  CLps22hhDriver() = default;
  ~CLps22hhDriver() = default;


  static constexpr uint8_t skuLps22hhRegIfCtrl_{ 0x0E };
  static constexpr uint8_t skuLps22hhRegWhoAmI_{ 0x0F };
  static constexpr uint8_t skuLps22hhRegCtrlReg1_{ 0x10 };
  static constexpr uint8_t skuLps22hhRegCtrlReg2_{ 0x11 };
  static constexpr uint8_t skuLps22hhRegCtrlReg3_{ 0x12 };
  static constexpr uint8_t skuLps22hhRegIntSource_{ 0x24 };
  static constexpr uint8_t skuLps22hhRegStatus_{ 0x27 };
  static constexpr uint8_t skuLps22hhRegPressOutXL_{ 0x28 };


  static constexpr uint8_t skuLps22hhChipId_{ 0xB3 };

  // Interface control register IF_CTRL (0Eh)
  static constexpr uint8_t skuLps22hhI2cDisable_{ 0x01 };
  static constexpr uint8_t skuLps22hhI3cDisable_{ 0x01 << 1U };
  static constexpr uint8_t skuLps22hhRegIfCtrlValue_{ skuLps22hhI2cDisable_ | skuLps22hhI3cDisable_ };

  // Control register 1 CTRL_REG1 (10h)
  static constexpr uint8_t skuLps22hhOdr25_{ 0x03 << 4U }; // ODR 25 Hz; Low-pass filter disabled; continuous update
  // Control register 2 CTRL_REG2 (11h)
  static constexpr uint8_t skuLps22hhLowNoiseEn_{ 0x01 << 1U };
  static constexpr uint8_t skuLps22hhSwReset_{ 0x01 << 2U };
  static constexpr uint8_t skuLps22hhIfAddInc_{ 0x01 << 4U };
  static constexpr uint8_t skuLps22hhBoot_{ 0x01 << 7U };
  static constexpr uint8_t skuLps22hhRegCtrlReg2Value_{ skuLps22hhLowNoiseEn_ | skuLps22hhIfAddInc_ };
  // Control register 3 - INT_DRDY pin control register CTRL_REG3 (12h)
  static constexpr uint8_t skuLps22hhDrdy_{ 0x01 << 2U };
  // Interrupt source INT_SOURCE (24h)
  static constexpr uint8_t skuLps22hhBootOn_{ 0x01 << 7U };
  // Status register STATUS (27h)
  static constexpr uint8_t skuLps22hhPressDataAvailable_{ 0x01 };
  static constexpr uint8_t skuLps22hhTempDataAvailable_{ 0x01 << 1U };
  static constexpr uint8_t skuLps22hhPressTempDataAvailable_{ (skuLps22hhPressDataAvailable_ | skuLps22hhTempDataAvailable_)};

  static constexpr float skfPaPerHPa_{ 100.0F };
  static constexpr float skfLps22hhPressSensitivityHPa_{ (1.0F / 4096.0F) }; // Table 3: Pressure sensitivity, hPa/LSB
  static constexpr float skfLps22hhPressSensitivityPa_{ (skfLps22hhPressSensitivityHPa_ * skfPaPerHPa_) }; // Pressure sensitivity, Pa/LSB

  static constexpr float skfLps22hhTempSensitivity_{ (1.0F / 100.0F) }; // Table 3: Temperature sensitivity, degC/LSB


  static bool getChipId(uint8_t& urChipId);
  static bool performSoftwareReset();
  static bool isBaroDataReady();
  static bool readPressureAndTemperature(float& fPressure, float& fTemperature);

  static bool readReg(uint8_t uRegAddr, uint8_t *upBuffer, uint16_t uLen);
  static bool writeReg(uint8_t uRegAddr, const uint8_t *upBuffer, uint16_t uLen);

  bool bIsInitialized_{ false };  ///< Sensor status after initialization

};

#endif /* C_LPS22HH_DRIVER_H */
