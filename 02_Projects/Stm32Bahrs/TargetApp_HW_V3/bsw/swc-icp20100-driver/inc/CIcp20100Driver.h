/// @file CIcp20100Driver.h
/// @brief Implementation of the ICP20100 driver software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_ICP20100_DRIVER_H
#define C_ICP20100_DRIVER_H

#include "General/CSoftwareComponentBase.h"


class CIcp20100Driver : public CSoftwareComponent<CIcp20100Driver, 1U>
{
  friend class CSoftwareComponent<CIcp20100Driver, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CIcp20100Driver)
  DECLARE_MANDATORY_APIS(CIcp20100Driver)

public:
  void PollSensor(uint64_t uTimestampUs);

private:
  CIcp20100Driver() = default;
  ~CIcp20100Driver() = default;

  static constexpr uint8_t skuIcp20100RegDeviceId_{ 0x0C };

  static constexpr uint8_t skuIcp20100RegModeSelect_{ 0xC0 };
  static constexpr uint8_t skuIcp20100RegInterruptStatus_{ 0xC1 };
  static constexpr uint8_t skuIcp20100RegInterruptMask_{ 0xC2 };
  static constexpr uint8_t skuIcp20100RegFifoConfig_{ 0xC3 };
  static constexpr uint8_t skuIcp20100RegFifoFill_{ 0xC4 };
  static constexpr uint8_t skuIcp20100RegDeviceStatus_{ 0xCD };

  static constexpr uint8_t skuIcp20100RegVersion_{ 0xD3 };
  static constexpr uint8_t skuIcp20100RegPressData0_{ 0xFA };

  static constexpr uint8_t skuIcp20100DeviceId_{ 0x63 };
  static constexpr uint8_t skuIcp20100VersionB2_{ 0xB2 }; // Version B, p. 53

  // MODE_SELECT register (C0h)
  // Continuous Measurements (duty cycled): Measurements are started based on the selected mode ODR_REG
  static constexpr uint8_t skuIcp20100MeasModeContinuous_{ 0x01 << 3U };

  // INTERRUPT_STATUS register (C1h)
  static constexpr uint8_t skuIcp20100InterruptStatusMask_{ 0x01 << 2U }; // (0x04) FIFO watermark high

  // INTERRUPT_MASK register (C2h)
  // Reset value: 0x00
  // Bit 7 -> 1 Reserved (program to 1)
  // Bit 6 -> 1 Masked
  // Bit 5 -> 1 Masked
  // Bit 4 -> 0 Reserved
  // Bit 3 -> 1 Masked
  // Bit 2 -> 0 FIFO_WMK_HIGH_MASK (0: FIFO_WMK_HIGH interrupt is not masked)
  // Bit 1 -> 1 Masked
  // Bit 0 -> 1 Masked
  static constexpr uint8_t skuIcp20100FifoWmkHighUnMask_{ 0xEB };

  // FIFO_CONFIG register (C3h)
  // FIFO high watermark value
  // Interrupt is triggered when the FIFO fill level reaches this value in the upward direction.
  // A value of 0 disables the high watermark check.
  static constexpr uint8_t skuIcp20100FifoWmHigh_{ 0x01 << 4U };

  // FIFO_FILL register (C4h)
  static constexpr uint8_t skuIcp20100FifoFlush_{ 0x01 << 7U };
  static constexpr uint8_t skuIcp20100FifoEmpty_{ 0x01 << 6U };

  // DEVICE_STATUS register (CDh)
  static constexpr uint8_t skuIcp20100ModeSyncStatus_{ 0x01 }; // 1: Synchronization of the selected mode to the internal clock domain is finished

  // 4.2.4 Supported Commands (P.22)
  static constexpr uint8_t skuIcp20100CmdReadReg_{ 0x3C }; // Read from register command
  static constexpr uint8_t skuIcp20100CmdWriteReg_{ 0x33 }; // Write to register command

  static constexpr uint8_t skuSamplesToIgnore_{14U}; ///< ICP20100 FIR filter group delay: first 14 samples are invalid after startup.

  static bool getDeviceId(uint8_t& urChipId);
  static bool getVersion(uint8_t& urVersion);
  static bool isWatermarkHighInterruptTriggered();
  static bool watermarkHighInterruptClear();
  static bool fifoFlush();
  static bool readPressureAndTemperature(float& frPressure, float& frTemperature);

  static bool readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen);
  static bool writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen);

  bool bIsInitialized_{ false };  ///< Sensor status after initialization
  uint8_t uIgnoredSamplesCount_{0U};
};

#endif /* C_ICP20100_DRIVER_H */
