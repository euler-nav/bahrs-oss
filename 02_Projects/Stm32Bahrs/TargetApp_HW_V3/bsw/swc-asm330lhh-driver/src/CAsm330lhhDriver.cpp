/**
 * @file CAsm330lhhDriver.cpp
 * @brief Implementation of the Asm330lhh driver software component class.
 * @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "CRte.h"
#include "CAsm330lhhDriver.h"
#include "AmsAssert.h"
#include "spi.h"

#ifdef ENABLE_PRINTF

#include <stdio.h>

#endif // ENABLE_PRINTF

CAsm330lhhDriver& CAsm330lhhDriver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CAsm330lhhDriver soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

bool CAsm330lhhDriver::IsInitialized()
{
  return bIsInitialized_;
}

void CAsm330lhhDriver::PollSensor(uint64_t uTimestamp)
{
  bool bStatus = false;
  uint8_t uStatusReg{};
  float fTemperature{};
  SImuMeasurement oOutput{};

  if (IsInitialized())
  {
    oOutput.uTimestampUs_ = uTimestamp;

    bStatus = readStatusRegister(uStatusReg);

    if (true == bStatus)
    {
      // Poll status register gyroscope and accelerometer data ready bits
      if ((uStatusReg & skuAsm330lhhAccelGyroDataReadyStat_) == skuAsm330lhhAccelGyroDataReadyStat_) // @208 Hz
      {
        CSpecificForce oSpecificForce{};
        CAngularRate oAngularRate{};

        bStatus = readAccelAndGyro(oSpecificForce, oAngularRate);

        if (true == bStatus)
        {
          oOutput.fSpecificForceX_ = oSpecificForce[0];
          oOutput.fSpecificForceY_ = oSpecificForce[1];
          oOutput.fSpecificForceZ_ = oSpecificForce[2];

          oOutput.fAngularRateX_ = oAngularRate[0];
          oOutput.fAngularRateY_ = oAngularRate[1];
          oOutput.fAngularRateZ_ = oAngularRate[2];

          oOutput.uImuValid_ = BoolToUint(true);
        }
      }
      // Poll status register temperature data ready bit
      if ((uStatusReg & skuAsm330lhhTempDataReadyStat_) == skuAsm330lhhTempDataReadyStat_) // @~52.8 Hz
      {
        bStatus = readTemp(fTemperature);

        if (true == bStatus)
        {
          oOutput.fTemperature_ = fTemperature;
          oOutput.uTemperatureValid_ = BoolToUint(true);
        }
      }
    }

    CRte::GetInstance().oPortImuInput3_.Write(oOutput);
  }
}

bool CAsm330lhhDriver::getChipID(uint8_t& urChipId)
{
  return readReg(skuAsm330lhhRegWhoAmI_, &urChipId, 1);
}

void CAsm330lhhDriver::Init()
{
  bool bStatus = false;
  uint8_t uChipId{};

  bStatus = getChipID(uChipId);
  if (true == bStatus)
  {
    if (uChipId != skuAsm330lhhChipId_)
    {
      bStatus = false;
    }
  }

  if (true == bStatus)
  {
    // Enable the proper device configuration
    bStatus = writeReg(skuAsm330lhhRegCtrl9Xl_, &skuAsm330lhhAccelDeviceConf_, 1);
  }

  if (true == bStatus)
  {
    // I2C interface disabled, Gyroscope LPF1 enabled
    bStatus = writeReg(skuAsm330lhhRegCtrl4C_, &skuAsm330lhhRegCtrl4cConfVal_, 1);
  }

  if (true == bStatus)
  {
    // Gyroscope LPF1 bandwidth selection (43Hz @ ODR = 208Hz)
    bStatus = writeReg(skuAsm330lhhRegCtrl6C_, &skfAsm330lhhLpf1BandwidthSel_, 1);
  }

  if (true == bStatus)
  {
    bStatus = configAccel();
  }

  if (true == bStatus)
  {
    bStatus = configGyro();
  }

  if (true == bStatus)
  {
    bStatus = configIntPin();
  }
  if (true == bStatus)
  {
    bIsInitialized_ = true;
  }

  //DEBUG
#ifdef ENABLE_PRINTF
  if (bStatus)
  {
    printf("ASM330LHH: Initialization finished\r\n");
  }
  else
  {
    printf("ASM330LHH: Initialization failed\r\n");
  }
#endif // ENABLE_PRINTF
}

bool CAsm330lhhDriver::configAccel(void)
{
  bool bStatus = false;

  // ODR = 208Hz; FS = 8g; Output from LPF2 second filtering stage selected LPF2_XL_EN = 1
  bStatus = writeReg(skuAsm330lhhRegCtrl1Xl_, &skuAsm330lhhAccelConfVal_, 1);

  if (true == bStatus)
  {
    // Low pass filter with ODR/4;
    bStatus = writeReg(skuAsm330lhhRegCtrl8Xl_, &skuAsm330lhhAccelFilterConfVal_, 1);
  }

  return bStatus;
}

bool CAsm330lhhDriver::configGyro(void)
{
  return writeReg(skuAsm330lhhRegCtrl2G_, &skuAsm330lhhGyroConfVal_, 1);
}

bool CAsm330lhhDriver::configIntPin(void)
{
  bool bStatus = false;
  uint8_t uRegValue{};

  bStatus = readReg(skuAsm330lhhRegCounterBdrReg1_, &uRegValue, 1);
  if (true == bStatus)
  {
    uRegValue |= skuAsm330lhhAccelDataReadyPulsed_; // Data-ready pulsed mode (the data ready pulses are 75 μs long)
    bStatus = writeReg(skuAsm330lhhRegCounterBdrReg1_, &uRegValue, 1);
  }

  // Register CTRL3_C (12h) not set. Interrupt output pins active high (push-pull mode)
  if (true == bStatus)
  {
    // Accelerometer data-ready interrupt on INT1 pin.
    bStatus = writeReg(skuAsm330lhhRegInt1Ctrl_, &skuAsm330lhhAccelDataReadyInt_, 1); // <- ACCEL
  }

  if (true == bStatus)
  {
    // Gyroscope data-ready interrupt on INT2 pin.
    bStatus = writeReg(skuAsm330lhhRegInt2Ctrl_, &skuAsm330lhhGyroDataReadyInt_, 1); // <- GYRO
  }

  return bStatus;
}

bool CAsm330lhhDriver::readStatusRegister(uint8_t& urStatusReg)
{
  return readReg(skuAsm330lhhRegStatusReg_, &urStatusReg, 1);
}

bool CAsm330lhhDriver::readAccelAndGyro(CSpecificForce& orSpecificForce, CAngularRate& orAngularRate)
{
  bool bStatus = false;
  uint8_t auValue[12];
  int16_t iAccelX{0}, iAccelY{0}, iAccelZ{0};
  int16_t iGyroX{0}, iGyroY{0}, iGyroZ{0};

  bStatus = readReg(skuAsm330lhhRegOutxLG_, auValue, 12);

  if (bStatus)
  {
    iGyroX = static_cast<int16_t>((auValue[1] << 8) | auValue[0]);
    iGyroY = static_cast<int16_t>((auValue[3] << 8) | auValue[2]);
    iGyroZ = static_cast<int16_t>((auValue[5] << 8) | auValue[4]);

    iAccelX = static_cast<int16_t>((auValue[7] << 8) | auValue[6]);
    iAccelY = static_cast<int16_t>((auValue[9] << 8) | auValue[8]);
    iAccelZ = static_cast<int16_t>((auValue[11] << 8) | auValue[10]);

    // Calculate angular rate
    orAngularRate[0] = static_cast<float>(iGyroX) * (skfGyroIntegerToFloatScaleFactor);
    orAngularRate[1] = static_cast<float>(iGyroY) * (skfGyroIntegerToFloatScaleFactor);
    orAngularRate[2] = static_cast<float>(iGyroZ) * (skfGyroIntegerToFloatScaleFactor);
    // Calculate specific force
    orSpecificForce[0] = static_cast<float>(iAccelX) * (skfAccelIntegerToFloatScaleFactor);
    orSpecificForce[1] = static_cast<float>(iAccelY) * (skfAccelIntegerToFloatScaleFactor);
    orSpecificForce[2] = static_cast<float>(iAccelZ) * (skfAccelIntegerToFloatScaleFactor);
  }

  return bStatus;
}

bool CAsm330lhhDriver::readTemp(float& fTemperature)
{
  bool bStatus = false;
  uint8_t auValue[2];

  bStatus = readReg(skuAsm330lhhRegOutTempL_, auValue, 2);
  if (true == bStatus)
  {
    const int16_t iTemp = static_cast<int16_t>((auValue[1] << 8) | auValue[0]);
    fTemperature = static_cast<float>(iTemp) * skfAsm330lhhTempSensitivity_ + 25.0F;
  }
  return bStatus;
}

bool CAsm330lhhDriver::readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen)
{
  uRegAddr = uRegAddr | 0x80;

  return CSpiHandler::GetInstance().Receive(CSpiHandler::EPeripheral::eAsm330, &uRegAddr, 1, upBuffer, uLen, 2);
}

bool CAsm330lhhDriver::writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen)
{
  return CSpiHandler::GetInstance().Transmit(CSpiHandler::EPeripheral::eAsm330, &uRegAddr, 1, upBuffer, uLen, 2);
}
