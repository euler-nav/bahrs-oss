/// @file CMmc5983Driver.cpp
/// @brief Implementation of the MMC5983 Driver software component
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CMmc5983Driver.h"
#include "CRte.h"
#include <math.h>
#include "GetMicroseconds.h"
#include "UintToBool.h"

#ifdef BAHRS_HW_V3
#include "CI2CHandler.h"
#else
#include "spi.h"
#endif // BAHRS_HW_V3


CMmc5983Driver& CMmc5983Driver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CMmc5983Driver soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CMmc5983Driver::Init()
{
  uint8_t uStatus = MMC5983_OK;

  setSoftReset();

  if(getProductID() != 0x30)
  {
    uStatus = MMC5983_ERR_CHIPID;
  }
  uSensorStatus_ = uStatus;

  if (true == IsInitialized())
  {
    setFilterBandwidth(EBandwidth::e100Hz);
    setContinuousModeFrequency(EMeasurementMode::e10Hz);
    // Enable automatic set/reset feature and enable the interrupt for completed measurements
    writeToAddress(MMC5983_REG_CONTROL0, MMC5983_AUTO_SR_EN | MMC5983_INT_MEAS_DONE_EN);
  }
}

void CMmc5983Driver::PollSensor(uint64_t uTimestamp)
{
  SMagneticMeasurement oOutput;

  if (true == IsInitialized())
  {
    uint8_t uStatusRegValue = getStatusRegValue();

    if(uStatusRegValue & MMC5983_MEAS_M_DONE)
    {
      // Clear Meas_M_Done & Meas_T_Done interrupts
      uStatusRegValue |= (MMC5983_MEAS_M_DONE | MMC5983_MEAS_T_DONE);
      writeToAddress(MMC5983_REG_STATUS, uStatusRegValue);

      oOutput.uTimestampUs_ = uTimestamp;
      getMeasurementXYZ(oOutput.fVectorX_, oOutput.fVectorY_, oOutput.fVectorZ_);
      oOutput.uValid_ = BoolToUint(true);
    }
    else
    {
      oOutput.uValid_ = BoolToUint(false);
    }
  }
  else
  {
    oOutput.uValid_ = BoolToUint(false);
  }

  CRte::GetInstance().oPortMagnetometerInput1_.Write(oOutput);
}

void CMmc5983Driver::PollSensor()
{
  PollSensor(GetMicroseconds());
}

bool CMmc5983Driver::PollSensorDma()
{
  return CI2CHandler::GetInstance().MemReadDma(CI2CHandler::EBus::eI2c1,
                                               MMC5983_I2C_ADDR,
                                               static_cast<uint16_t>(MMC5983_REG_X_OUT0),
                                               1U,
                                               auDmaBuffer_.data(),
                                               static_cast<uint16_t>(auDmaBuffer_.size()));
}

void CMmc5983Driver::ProcessReceivedBytesDma(uint64_t uTimestampUs)
{
  SMagneticMeasurement oOutput{};

  if (true == IsInitialized())
  {
    // Clear the interrupt status. This is required.
    const uint8_t kuClearStatus = auDmaBuffer_[8] | (MMC5983_MEAS_M_DONE | MMC5983_MEAS_T_DONE);
    writeToAddress(MMC5983_REG_STATUS, kuClearStatus);

    oOutput.uTimestampUs_ = uTimestampUs;
    decodeMeasurementPayload(auDmaBuffer_, oOutput.fVectorX_, oOutput.fVectorY_, oOutput.fVectorZ_);
    oOutput.uValid_ = BoolToUint(true);
  }

  CRte::GetInstance().oPortMagnetometerInput1_.Write(oOutput);
}

void CMmc5983Driver::InvalidatePortData()
{
  SMagneticMeasurement oOutput{};
  CRte::GetInstance().oPortMagnetometerInput1_.Write(oOutput);
}

uint8_t CMmc5983Driver::getProductID()
{
  uint8_t uCtrl = 0;
  readFromAddress(&uCtrl, MMC5983_REG_PRODUCT_ID, 1);
  return uCtrl;
}

void CMmc5983Driver::getMeasurementXYZ(float& frValueX, float& frValueY, float& frValueZ)
{
  CI2CReadPayload oReadPayload{};
  readFromAddress(oReadPayload.begin(), MMC5983_REG_X_OUT0, oReadPayload.size());
  decodeMeasurementPayload(oReadPayload, frValueX, frValueY, frValueZ);
}

void CMmc5983Driver::decodeMeasurementPayload(const CI2CReadPayload& korPayload,
                                              float& frValueX,
                                              float& frValueY,
                                              float& frValueZ)
{
  float fData18bit[3]{};
  uint32_t uTmp{0};
  uint32_t uResult{0};

  // Get 18bits data, raw data unit is "count or LSB"
  uTmp = static_cast<uint32_t>(korPayload[0]);
  uTmp = uTmp << MMC5983_XYZ_0_SHIFT;
  uResult |= uTmp;

  uTmp = static_cast<uint32_t>(korPayload[1]);
  uTmp = uTmp << MMC5983_XYZ_1_SHIFT;
  uResult |= uTmp;

  uTmp = static_cast<uint32_t>(korPayload[6]);
  uTmp &= MMC5983_X2_MASK;
  uTmp = uTmp >> 6;
  uResult |= uTmp;

  fData18bit[0] = static_cast<float>(uResult);

  uResult = 0;
  uTmp = static_cast<uint32_t>(korPayload[2]);
  uTmp = uTmp << MMC5983_XYZ_0_SHIFT;
  uResult |= uTmp;

  uTmp = static_cast<uint32_t>(korPayload[3]);
  uTmp = uTmp << MMC5983_XYZ_1_SHIFT;
  uResult |= uTmp;

  uTmp = static_cast<uint32_t>(korPayload[6]);
  uTmp &= MMC5983_Y2_MASK;
  uTmp = uTmp >> 4;
  uResult |= uTmp;

  fData18bit[1] = static_cast<float>(uResult);

  uResult = 0;
  uTmp = static_cast<uint32_t>(korPayload[4]);
  uTmp = uTmp << MMC5983_XYZ_0_SHIFT;
  uResult |= uTmp;

  uTmp = static_cast<uint32_t>(korPayload[5]);
  uTmp = uTmp << MMC5983_XYZ_1_SHIFT;
  uResult |= uTmp;

  uTmp = static_cast<uint32_t>(korPayload[6]);
  uTmp &= MMC5983_Z2_MASK;
  uTmp = uTmp >> 2;
  uResult |= uTmp;

  fData18bit[2] = static_cast<float>(uResult);

  // Magnetic field output, unit is Gauss
  const float kfTmpValueX = (fData18bit[0] - MMC5983_18BIT_OFFSET) / MMC5983_18BIT_SENSITIVITY;
  const float kfTmpValueY = (fData18bit[1] - MMC5983_18BIT_OFFSET) / MMC5983_18BIT_SENSITIVITY;
  const float kfTmpValueZ = (fData18bit[2] - MMC5983_18BIT_OFFSET) / MMC5983_18BIT_SENSITIVITY;

#ifdef BAHRS_HW_V3
  frValueX = kfTmpValueX;
  frValueY = kfTmpValueY;
  frValueZ = -kfTmpValueZ;
#else
  frValueX = -kfTmpValueY;
  frValueY = kfTmpValueX;
  frValueZ = -kfTmpValueZ;
#endif // BAHRS_HW_V3
}

float CMmc5983Driver::getTemperature()
{
  uint8_t uRegStatus = 0U;
  uint8_t uRegT = 0U;
  float fTemperature = 0.0;

  // Write reg 0x1B, set TM_T bit '1', Take temperature measurement
  writeToAddress(MMC5983_REG_CONTROL0, MMC5983_CMD_TMT);
  HAL_Delay(1);

  // Read Meas_T_done bit
  readFromAddress(&uRegStatus, MMC5983_REG_STATUS, 1);
  while((uRegStatus & MMC5983_MEAS_T_DONE) != MMC5983_MEAS_T_DONE)
  {
    readFromAddress(&uRegStatus, MMC5983_REG_STATUS, 1);
  }

  // Read reg 0x07
  readFromAddress(&uRegT, MMC5983_REG_T_OUT0, 1);

  // Temperature output, unit is degree Celsius
  // The temperature output has not been calibrated, can not present the ambient temperature
  fTemperature = static_cast<float>(uRegT) * MMC5983_T_SENSITIVITY + MMC5983_T_ZERO;
  return fTemperature;
}

void CMmc5983Driver::setSoftReset()
{
  writeToAddress(MMC5983_REG_CONTROL1, MMC5983_SW_RST);
  HAL_Delay(11);
}

void CMmc5983Driver::setBitReset()
{
  writeToAddress(MMC5983_REG_CONTROL0, MMC5983_BIT_RESET);
}

void CMmc5983Driver::setBitSet()
{
  writeToAddress(MMC5983_REG_CONTROL0, MMC5983_BIT_SET);
}

void CMmc5983Driver::setContinuousModeFrequency(EMeasurementMode eMeasurementMode)
{
  uint8_t uVal = static_cast<uint8_t>(eMeasurementMode);
  writeToAddress(MMC5983_REG_CONTROL2, uVal | MMC5983_CM_EN);
}

void CMmc5983Driver::setFilterBandwidth(EBandwidth eBandwidth)
{
  uint8_t uVal = static_cast<uint8_t>(eBandwidth);
  writeToAddress(MMC5983_REG_CONTROL1, uVal);
}

void CMmc5983Driver::setAutoSetReset()
{
  writeToAddress(MMC5983_REG_CONTROL0, MMC5983_AUTO_SR_EN);
}

bool CMmc5983Driver::IsInitialized()
{
  return (MMC5983_OK == uSensorStatus_) ? true : false;
}

uint8_t CMmc5983Driver::getStatusRegValue()
{
  uint8_t uRes = 0;
  readFromAddress(&uRes, MMC5983_REG_STATUS, 1);
  return uRes;
}

#ifdef BAHRS_HW_V3

void CMmc5983Driver::writeToAddress(uint8_t uWriteAddr, uint8_t uVal)
{
  (void)CI2CHandler::GetInstance().MemWrite(CI2CHandler::EBus::eI2c1,
                                            MMC5983_I2C_ADDR,
                                            static_cast<uint16_t>(uWriteAddr),
                                            1U,
                                            &uVal,
                                            1U,
                                            skuReadWriteTimeoutI2c_);
}

void CMmc5983Driver::readFromAddress(uint8_t* upBuffer,  uint8_t uReadAddr,  uint16_t uNumByteToRead)
{
  (void)CI2CHandler::GetInstance().MemRead(CI2CHandler::EBus::eI2c1,
                                           MMC5983_I2C_ADDR,
                                           static_cast<uint16_t>(uReadAddr),
                                           1U,
                                           upBuffer,
                                           uNumByteToRead,
                                           skuReadWriteTimeoutI2c_);
}

#else

void CMmc5983Driver::writeToAddress(uint8_t uWriteAddr, uint8_t uVal)
{
  setCsOn();

  spiWriteRead(uWriteAddr);
  spiWriteRead(uVal);

  setCsOff();
}

void CMmc5983Driver::setCsOn()
{
  HAL_GPIO_WritePin(MMC_CSB_GPIO_Port, MMC_CSB_Pin, GPIO_PIN_RESET);
}

void CMmc5983Driver::setCsOff()
{
  HAL_GPIO_WritePin(MMC_CSB_GPIO_Port, MMC_CSB_Pin, GPIO_PIN_SET);
}

void CMmc5983Driver::readFromAddress(uint8_t *upBuffer,  uint8_t uReadAddr,  uint16_t uNumByteToRead)
{
  uint8_t uBuff = 0;

  uReadAddr |= MMC5983_READWRITE;
  setCsOn();

  spiWriteRead(uReadAddr);

  while(uNumByteToRead > 0x00)
  {
    uBuff = spiWriteRead(DUMMY_BYTE);
    *upBuffer = uBuff;
    uNumByteToRead--;
    upBuffer++;
  }
  setCsOff();
}

uint8_t CMmc5983Driver::spiWriteRead(uint8_t uByte)
{
  uint8_t uReceivedByte = 0;
  HAL_SPI_TransmitReceive(&hspi3, &uByte, &uReceivedByte, 1, skuReadWriteTimeoutSpi_);

  return uReceivedByte;
}

#endif // BAHRS_HW_V3
