/**
 * @file CBmp384Driver.cpp
 * @brief Implementation of the BMP384 Driver software component
 * @author Evgeniy Zamyatin
 * @date 26 April 2022
 */

#include "CBmp384Driver.h"
#include "CRte.h"
#include "main.h"
#include <math.h>
#include "GetMicroseconds.h"
#include "UintToBool.h"

extern SPI_HandleTypeDef hspi2;

CBmp384Driver& CBmp384Driver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CBmp384Driver soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CBmp384Driver::Init()
{
  uint8_t uRes = 0;
  uint8_t uStatus = BMP384_OK;

  if(getChipID() != 0x50)
  {
    uStatus |= BMP384_ERR_CHIPID;
  }

  setSoftReset();
  uRes = getEvent();

  if (uRes != 0x01)
  {
    uStatus |= BMP384_ERR_SOFTRESET;
  }

  uSensorStatus_ = uStatus;

  if (true == IsInitialized())
  {
    getTrimParam();
    setFilter(EIirFilter::eOff);
    setTimeStandby(ETimeStandby::e40MS);
    setOversamplingRegister(EOversampling::eX16, EOversampling::eX2);
    setPowerMode(EMode::eNormal);
  }

}

void CBmp384Driver::PollSensor()
{
  SBarometerMeasurement oOutput;

  if (true == IsInitialized())
  {
    //
    // Here we need to poll sensor data and compensate it
    //
    uint8_t uReadyRead = isReadyRead();

    if(uReadyRead & (BMP384_DRDY_TEMP | BMP384_DRDY_PRESS))  // Temperature and pressure data is ready read
    {
      oOutput.uTimestampUs_ = GetMicroseconds();
      getTemperatureAndPressure(oOutput.fTemperature_, oOutput.fPressure_);
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

  CRte::GetInstance().oPortBmp384Input_.Write(oOutput);
}

uint8_t CBmp384Driver::getChipID()
{
  uint8_t uCtrl = 0;
  readFromAddress(&uCtrl, BMP384_REG_CHIP_ID, 1);
  return uCtrl;
}

void CBmp384Driver::setSoftReset()
{
  writeToAddress(BMP384_REG_CMD, BMP384_SOFTRESET);
  HAL_Delay(2000);
}

uint8_t CBmp384Driver::getEvent()
{
  uint8_t uCtrl = 0;
  readFromAddress(&uCtrl, BMP384_REG_EVENT, 1);
  return uCtrl;
}

void CBmp384Driver::setPowerMode(EMode eMode)
{
  uint8_t uVal = 0;
  switch (eMode)
  {
    case EMode::eSleep:
      uVal = BMP384_SLEEP_MODE;
      break;
    case EMode::eForce:
      uVal = BMP384_FORCED_MODE;
      break;
    case EMode::eNormal:
      uVal = BMP384_NORMAL_MODE;
      break;
  }

  uVal |= (BMP384_TEMP_EN | BMP384_PRESS_EN);
  writeToAddress(BMP384_REG_PWR_CTRL, uVal);
}

uint8_t CBmp384Driver::getPowerControl()
{
  uint8_t uCtrl = 0;
  readFromAddress(&uCtrl, BMP384_REG_PWR_CTRL, 1);
  return uCtrl;
}

void CBmp384Driver::getTemperature(float &fTemperature)
{
  uint8_t auData[3];

  readFromAddress(auData, BMP384_REG_TEMP0, 3);
  int32_t iAdcTemp = (int32_t)auData[2] << 16 | (int32_t)auData[1] << 8 | (int32_t)auData[0];
  fTemperature = compensateTemperature(static_cast<float>(iAdcTemp));
}

void CBmp384Driver::getPressure(float &fPressure)
{
  float fTemperature;
  getTemperatureAndPressure(fTemperature, fPressure);
}

void CBmp384Driver::getTemperatureAndPressure(float &fTemperature, float &fPressure)
{
  uint8_t auData[6];

  readFromAddress(auData, BMP384_REG_PREES0, 6);
  int32_t iAdcTemp = (int32_t)auData[5] << 16 | (int32_t)auData[4] << 8 | (int32_t)auData[3];
  int32_t iAdcPres = (int32_t)auData[2] << 16 | (int32_t)auData[1] << 8 | (int32_t)auData[0];
  fTemperature = compensateTemperature(static_cast<float>(iAdcTemp));
  fPressure = compensatePressure(static_cast<float>(iAdcPres), fTemperature);
}

void CBmp384Driver::getTrimParam()
{
  float fVal = 0.0F;

  readFromAddress(reinterpret_cast<uint8_t*>(&oParams_), BMP384_REG_TRIM, sizeof(SParams));

  oFloatParams_.fT1_ = ldexpf(static_cast<float>(oParams_.uT1_), 8);
  oFloatParams_.fT2_ = ldexpf(static_cast<float>(oParams_.uT2_), -30);
  oFloatParams_.fT3_ = ldexpf(static_cast<float>(oParams_.uT3_), -48);

  fVal = static_cast<float>(oParams_.uP1_) - ldexpf(1.0F, 14);
  oFloatParams_.fP1_ = ldexpf(fVal, -20);

  fVal = static_cast<float>(oParams_.uP2_) - ldexpf(1.0F, 14);
  oFloatParams_.fP2_ = ldexpf(fVal, -29);

  oFloatParams_.fP3_ = ldexpf(static_cast<float>(oParams_.uP3_), -32);
  oFloatParams_.fP4_ = ldexpf(static_cast<float>(oParams_.uP4_), -37);
  oFloatParams_.fP5_ = ldexpf(static_cast<float>(oParams_.uP5_), 3);
  oFloatParams_.fP6_ = ldexpf(static_cast<float>(oParams_.uP6_), -6);
  oFloatParams_.fP7_ = ldexpf(static_cast<float>(oParams_.uP7_), -8);
  oFloatParams_.fP8_ = ldexpf(static_cast<float>(oParams_.uP8_), -15);
  oFloatParams_.fP9_ = ldexpf(static_cast<float>(oParams_.uP9_), -48);
  oFloatParams_.fP10_ = ldexpf(static_cast<float>(oParams_.uP10_), -48);
  oFloatParams_.fP11_ = ldexpf(static_cast<float>(oParams_.uP11_), -65);
}

uint8_t CBmp384Driver::getSensorStatus()
{
  return uSensorStatus_;
}

void CBmp384Driver::setFilter(EIirFilter eFilterType)
{
  uint8_t uVal = static_cast<uint8_t>(eFilterType);
  uVal = uVal << 1;
  writeToAddress(BMP384_REG_CONFIG, uVal);
}

void CBmp384Driver::setTimeStandby(ETimeStandby eTimeStandby)
{
  uint8_t uVal = static_cast<uint8_t>(eTimeStandby);
  writeToAddress(BMP384_REG_ODR, uVal);
}

void CBmp384Driver::setOversamplingRegister(EOversampling ePresOversampling, EOversampling eTempOversampling)
{
  uint8_t uVal1 = static_cast<uint8_t>(ePresOversampling);
  uint8_t uVal2 = static_cast<uint8_t>(eTempOversampling);

  uint8_t val =  uVal2 << 3 |  uVal1;
  writeToAddress(BMP384_REG_OSR, val);
}

void CBmp384Driver::setPresOversampling(EOversampling ePresOversampling)
{
  uint8_t uVal = static_cast<uint8_t>(ePresOversampling);
  writeToAddress(BMP384_REG_OSR, uVal);
}

void CBmp384Driver::setTempOversampling(EOversampling eTempOversampling)
{
  uint8_t uVal = static_cast<uint8_t>(eTempOversampling);
  writeToAddress(BMP384_REG_OSR, uVal);
}

uint8_t CBmp384Driver::isReadyRead()
{
  uint8_t uRes = 0;
  readFromAddress(&uRes, BMP384_REG_STATUS, 1);
  return uRes;
}

bool CBmp384Driver::IsInitialized()
{
  return (BMP384_OK == uSensorStatus_) ? true : false;
}

void CBmp384Driver::readFromAddress(uint8_t *upBuffer,  uint8_t uReadAddr,  uint16_t uNumByteToRead)
{
  uint8_t uBuff = 0;

  uReadAddr |= (uint8_t)(READWRITE_CMD);

  setCsOn();

  spiWriteRead(uReadAddr);
  spiWriteRead(DUMMY_BYTE);

  while(uNumByteToRead > 0x00)
  {
    uBuff = spiWriteRead(DUMMY_BYTE);;
    *upBuffer = uBuff;
     uNumByteToRead--;
     upBuffer++;
  }
  setCsOff();
}

float CBmp384Driver::compensatePressure(float fUncompensatedPressure, float fCompensatedTemperature)
{
  float fPartialData1 = oFloatParams_.fP6_ * fCompensatedTemperature;
  float fPartialData2 = oFloatParams_.fP7_ * fCompensatedTemperature * fCompensatedTemperature;
  float fPartialData3 = oFloatParams_.fP8_ * fCompensatedTemperature * fCompensatedTemperature * fCompensatedTemperature;
  float fPartialOut1 = oFloatParams_.fP5_ + fPartialData1 + fPartialData2 + fPartialData3;

  fPartialData1 = oFloatParams_.fP2_ * fCompensatedTemperature;
  fPartialData2 = oFloatParams_.fP3_ * fCompensatedTemperature * fCompensatedTemperature;
  fPartialData3 = oFloatParams_.fP4_ * fCompensatedTemperature * fCompensatedTemperature * fCompensatedTemperature;

  float fPartialOut2 = fUncompensatedPressure * (oFloatParams_.fP1_ + fPartialData1 + fPartialData2 + fPartialData3);

  fPartialData1 = fUncompensatedPressure * fUncompensatedPressure;
  fPartialData2 = oFloatParams_.fP9_ + oFloatParams_.fP10_ * fCompensatedTemperature;
  fPartialData3 = fPartialData1 * fPartialData2;

  float fPartialData4 = fPartialData3 + fUncompensatedPressure * fUncompensatedPressure * fUncompensatedPressure * oFloatParams_.fP11_;

  return fPartialOut1 + fPartialOut2 + fPartialData4;
}

float CBmp384Driver::compensateTemperature(float fUncompensatedTemperature)
{
  float fPartialData1 = fUncompensatedTemperature - oFloatParams_.fT1_;
  float fPartialData2 = fPartialData1 * oFloatParams_.fT2_;
  return fPartialData2 + fPartialData1 * fPartialData1 * oFloatParams_.fT3_;
}

void CBmp384Driver::writeToAddress(uint8_t uWriteAddr, uint8_t uVal)
{
  setCsOn();

  spiWriteRead(uWriteAddr);
  spiWriteRead(uVal);

  setCsOff();
}

void CBmp384Driver::setCsOn()
{
  HAL_GPIO_WritePin(BMP384_CSB_GPIO_Port, BMP384_CSB_Pin, GPIO_PIN_RESET);
}

void CBmp384Driver::setCsOff()
{
  HAL_GPIO_WritePin(BMP384_CSB_GPIO_Port, BMP384_CSB_Pin, GPIO_PIN_SET);
}

uint8_t CBmp384Driver::spiWriteRead(uint8_t uByte)
{
  uint8_t uReceivedByte = 0;
  HAL_SPI_TransmitReceive(&hspi2, &uByte, &uReceivedByte, 1, 1);
  return uReceivedByte;
}

