/**
 * @file CMmc5983Driver.cpp
 * @brief Implementation of the MMC5983 Driver software component
 * @author Evgeniy Zamyatin
 * @date 31 May 2022
 */

#include "CMmc5983Driver.h"
#include "CRte.h"
#include "spi.h"
#include <math.h>
#include "GetMicroseconds.h"
#include "UintToBool.h"


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
    setBitSet();
    setFilterBandwidth(EBandwidth::e100Hz);
    setContinuousModeFrequency(EMeasurementMode::e10Hz);
  }
}

void CMmc5983Driver::PollSensor()
{
  SMagneticMeasurement oOutput;

  if (true == IsInitialized())
  {
    uint8_t uReadyRead = isReadyRead();

    if(uReadyRead & MMC5983_M_DONE)
    {
      oOutput.uTimestampUs_ = GetMicroseconds();
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

  CRte::GetInstance().oPortMmc5983Input_.Write(oOutput);
}

uint8_t CMmc5983Driver::getProductID()
{
  uint8_t uCtrl = 0;
  readFromAddress(&uCtrl, MMC5983_REG_PRODUCT_ID, 1);
  return uCtrl;
}

void CMmc5983Driver::getMeasurementXYZ(float& frValueX, float& frValueY, float& frValueZ)
{
  uint8_t uDataReg[7] = {0};
  float fData18bit[3] = {0};
  uint32_t uTemp = 0;
  uint32_t uResult = 0;
  readFromAddress(uDataReg, MMC5983_REG_X_OUT0, 7);

  /* Get 18bits data, raw data unit is "count or LSB" */
  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_X_OUT0]);
  uTemp = uTemp << MMC5983_XYZ_0_SHIFT;
  uResult |= uTemp;

  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_X_OUT1]);
  uTemp = uTemp << MMC5983_XYZ_1_SHIFT;
  uResult |= uTemp;

  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_XYZ_OUT2]);
  uTemp &= MMC5983_X2_MASK;
  uTemp = uTemp >> 6;
  uResult |= uTemp;

  fData18bit[0] = static_cast<float>(uResult);

  uResult = 0;
  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_Y_OUT0]);
  uTemp = uTemp << MMC5983_XYZ_0_SHIFT;
  uResult |= uTemp;

  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_Y_OUT1]);
  uTemp = uTemp << MMC5983_XYZ_1_SHIFT;
  uResult |= uTemp;

  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_XYZ_OUT2]);
  uTemp &= MMC5983_Y2_MASK;
  uTemp = uTemp >> 4;
  uResult |= uTemp;

  fData18bit[1] = static_cast<float>(uResult);

  uResult = 0;
  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_Z_OUT0]);
  uTemp = uTemp << MMC5983_XYZ_0_SHIFT;
  uResult |= uTemp;

  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_Z_OUT1]);
  uTemp = uTemp << MMC5983_XYZ_1_SHIFT;
  uResult |= uTemp;

  uTemp = static_cast<uint32_t>(uDataReg[MMC5983_REG_XYZ_OUT2]);
  uTemp &= MMC5983_Z2_MASK;
  uTemp = uTemp >> 2;
  uResult |= uTemp;

  fData18bit[2] = static_cast<float>(uResult);

  /* Magnetic field output, unit is Gauss */
  float fTmpValueX = (fData18bit[0] - MMC5983_18BIT_OFFSET) / MMC5983_18BIT_SENSITIVITY;
  float fTmpValueY = (fData18bit[1] - MMC5983_18BIT_OFFSET) / MMC5983_18BIT_SENSITIVITY;
  float fTmpValueZ = (fData18bit[2] - MMC5983_18BIT_OFFSET) / MMC5983_18BIT_SENSITIVITY;

  /* Sensor-frame to body-frame output conversion:
   *
   *        /  0 -1  0 \
   * C_bs = |  1  0  0 |
   *        \  0  0 -1 /
  */

  frValueX = -fTmpValueY;
  frValueY = fTmpValueX;
  frValueZ = -fTmpValueZ;
}

float CMmc5983Driver::getTemperature()
{
  uint8_t uRegStatus = 0U;
  uint8_t uRegT = 0U;
  float fTemperature = 0.0;

  /* Write reg 0x1B, set TM_T bit '1', Take temperature measurement */
  writeToAddress(MMC5983_REG_CONTROL0,MMC5983_CMD_TMT);
  HAL_Delay(1);

  /* Read Meas_T_done bit */
  readFromAddress(&uRegStatus, MMC5983_REG_STATUS, 1);
  while((uRegStatus & MMC5983_T_DONE) != MMC5983_T_DONE)
  {
    readFromAddress(&uRegStatus, MMC5983_REG_STATUS, 1);
  }

  /* Read reg 0x07 */
  readFromAddress(&uRegT, MMC5983_REG_T_OUT0, 1);

  /* Temperature output, unit is degree Celsius */
  /* The temperature output has not been calibrated, can not present the ambient temperature*/
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

uint8_t CMmc5983Driver::isReadyRead()
{
  uint8_t uRes = 0;
  readFromAddress(&uRes, MMC5983_REG_STATUS, 1);
  return uRes;
}

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
  HAL_SPI_TransmitReceive(&hspi3, &uByte, &uReceivedByte, 1, 1);

  return uReceivedByte;
}
