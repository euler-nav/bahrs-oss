/**
 * @file CAsm330lhhDriver.h
 * @brief Implementation of the Asm330lhh driver software component class.
 * @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

// Used ASM330LHH datasheet revision: DocID031239 Rev 4 (May 2020)

#ifndef C_ASM330LHH_DRIVER_H
#define C_ASM330LHH_DRIVER_H

#include "General/CSoftwareComponentBase.h"

class CAsm330lhhDriver : public CSoftwareComponent<CAsm330lhhDriver, 1U>
{
  friend class CSoftwareComponent<CAsm330lhhDriver, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CAsm330lhhDriver)
  DECLARE_MANDATORY_APIS(CAsm330lhhDriver)

public:
  void PollSensor(uint64_t uTimestamp);

private:
  CAsm330lhhDriver() = default;
  ~CAsm330lhhDriver() = default;

  static constexpr uint8_t skuAsm330lhhRegCounterBdrReg1_{ 0x0B };
  static constexpr uint8_t skuAsm330lhhRegInt1Ctrl_{ 0x0D };
  static constexpr uint8_t skuAsm330lhhRegInt2Ctrl_{ 0x0E };
  static constexpr uint8_t skuAsm330lhhRegWhoAmI_{ 0x0F };
  static constexpr uint8_t skuAsm330lhhRegCtrl1Xl_{ 0x10 };
  static constexpr uint8_t skuAsm330lhhRegCtrl2G_{ 0x11 };
  static constexpr uint8_t skuAsm330lhhRegCtrl4C_{ 0x13 };
  static constexpr uint8_t skuAsm330lhhRegCtrl6C_{ 0x15 };
  static constexpr uint8_t skuAsm330lhhRegCtrl8Xl_{ 0x17 };
  static constexpr uint8_t skuAsm330lhhRegCtrl9Xl_{ 0x18 };
  static constexpr uint8_t skuAsm330lhhRegStatusReg_{ 0x1E };
  static constexpr uint8_t skuAsm330lhhRegOutTempL_{ 0x20 };
  static constexpr uint8_t skuAsm330lhhRegOutxLG_{ 0x22 };


  static constexpr uint8_t skuAsm330lhhChipId_{ 0x6B };

  // Counter batch data rate register 1 (0Bh)
  static constexpr uint8_t skuAsm330lhhAccelDataReadyPulsed_{ 0x01 << 7U};

  // INT1 pin control register INT1_CTRL (0Dh)
  // INT2 pin control register INT2_CTRL (0Eh)
  static constexpr uint8_t skuAsm330lhhAccelDataReadyInt_{ 0x01 };
  static constexpr uint8_t skuAsm330lhhGyroDataReadyInt_{ (0x01 << 1U) };
  static constexpr uint8_t skuAsm330lhhTempDataReadyInt_{ (0x01 << 2U) };

  // Accelerometer control register 1 CTRL1_XL (10h)
  static constexpr uint8_t skuAsm330lhhAccelOdr208_{ (0x05 << 4U) }; // p. 49
  static constexpr uint8_t skuAsm330lhhAccelFs8G_{ (0x03 << 2U) }; // p. 49
  static constexpr uint8_t skuAsm330lhhAccelLpf2_{ (0x01 << 1U) }; // Output from LPF2 second filtering stage selected (LPF2_XL_EN = 1) (p. 32, 49)

  static constexpr uint8_t skuAsm330lhhAccelConfVal_{ (skuAsm330lhhAccelOdr208_ | skuAsm330lhhAccelFs8G_ | skuAsm330lhhAccelLpf2_) };

  // Accelerometer control register 8 (17h)
  static constexpr uint8_t skuAsm330lhhAccelFilterConfVal_{ 0 }; // HP_SLOPE_XL_EN = 0; Low pass with ODR/4 (p. 55, 56)

  // Accelerometer control register 9 (18h)
  static constexpr uint8_t skuAsm330lhhAccelDeviceConf_{ (0x01 << 1U) }; // p. 38, 57

  // Gyroscope control register 2 CTRL2_G (11h)
  static constexpr uint8_t skuAsm330lhhGyroOdr208_{ (0x05 << 4U) };
  static constexpr uint8_t skuAsm330lhhGyroRange500_{ (0x01 << 2U) };

  static constexpr uint8_t skuAsm330lhhGyroConfVal_{ (skuAsm330lhhGyroOdr208_ | skuAsm330lhhGyroRange500_) };

  // Control register 4 CTRL4_C (13h)
  static constexpr uint8_t skfAsm330lhhI2cDisable_{ (0x01 << 2U) };
  static constexpr uint8_t skfAsm330lhhLpf1Enable_{ (0x01 << 1U) };
  static constexpr uint8_t skuAsm330lhhRegCtrl4cConfVal_{ skfAsm330lhhI2cDisable_ | skfAsm330lhhLpf1Enable_ };

  // Control register 6 CTRL6_C (15h)
  static constexpr uint8_t skfAsm330lhhLpf1BandwidthSel_{ 0x05};

  // Status register STATUS_REG (1Eh)
  static constexpr uint8_t skuAsm330lhhAccelDataReadyStat_{ 0x01 };
  static constexpr uint8_t skuAsm330lhhGyroDataReadyStat_{ (0x01 << 1U) };
  static constexpr uint8_t skuAsm330lhhTempDataReadyStat_{ (0x01 << 2U) };
  static constexpr uint8_t skuAsm330lhhAccelGyroDataReadyStat_{ (skuAsm330lhhAccelDataReadyStat_ | skuAsm330lhhGyroDataReadyStat_) };

  // Gyroscope
  static constexpr float skfAsm330lhhGyroSensitivity_{ (17.5F / 1000.0F) }; // dps/LSB (Table 3: Angular rate sensitivity)
  static constexpr float skfDegToRad_{ (3.141592F / 180.0F) };
  static constexpr float skfGyroIntegerToFloatScaleFactor{ skfAsm330lhhGyroSensitivity_ * skfDegToRad_ };

  // Accelerometer
  static constexpr float skfAsm330lhhAccelSensitivity_{ (0.244F / 1000.0F) }; // g/LSB (Table 3: Linear acceleration sensitivity)
  static constexpr float skfStandardGravity_{ 9.80665F };
  static constexpr float skfAccelIntegerToFloatScaleFactor{ skfAsm330lhhAccelSensitivity_ * skfStandardGravity_ };

  // Temperature
  static constexpr float skfAsm330lhhTempSensitivity_{ (1.0F / 256.0F) }; // Table 5: Temperature sensitivity


  static bool getChipID(uint8_t& urChipId);
  static bool enableGyroAccelTemp();
  static bool configAccel();
  static bool configGyro();
  static bool configIntPin();
  static bool readStatusRegister(uint8_t& urStatusReg);

  using CSpecificForce = std::array<float, 3>;
  using CAngularRate = std::array<float, 3>;

  static bool readAccelAndGyro(CSpecificForce& orSpecificForce, CAngularRate& orAngularRate);
  static bool readTemp(float& fTemperature);
  static bool readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen);
  static bool writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen);

  bool bIsInitialized_{ false };  ///< Sensor status after initialization

};

#endif /* C_ASM330LHH_DRIVER_H */
