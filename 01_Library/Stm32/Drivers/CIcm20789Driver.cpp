/**
 * @file CIcm20789Driver.cpp
 * @brief Implementation of the ICM20789 Driver software component
 * @author Evgeniy Zamyatin
 * @date 22 June 2022
 */

#include "CIcm20789Driver.h"
#include "CRte.h"
#include <math.h>
#include "GetMicroseconds.h"
#include "UintToBool.h"


CIcm20789Driver::CIcm20789Driver(EIcmIds eIcmChipId) :
  keSensorId_(eIcmChipId),
  opkI2CHandle_(getI2CHandle(eIcmChipId))
{
  if (EIcmIds::eInvalid == keSensorId_)
  {
    // We must not instantiate an invalid driver.
    assert(false);
  }
}

CIcm20789Driver& CIcm20789Driver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CIcm20789Driver soInstance1(EIcmIds::eIcm1), soInstance2(EIcmIds::eIcm2);

  assert(uInstanceIndex < CIcm20789Driver::skuInstanceCount_);

  CIcm20789Driver* opInstance;

  if (0U == uInstanceIndex)
  {
    opInstance = &soInstance1;
  }
  else if (1U == uInstanceIndex)
  {
    opInstance = &soInstance2;
  }
  else
  {
    assert(false);
  }

  return *opInstance;
}

void CIcm20789Driver::Init()
{
  bool bStatus;
  uint8_t uBaroId = 0U;
  uint8_t uImuId = 0U;

  bStatus = softResetImu();

  // Check IMU ID
  if(true == bStatus)
  {
    bStatus = getImuChipId(uImuId);
  }

  if (true == bStatus)
  {
    if (uImuId != ICM20789_IMU_ID)
    {
      bStatus = false;
    }
  }

  // Initialize the IMU
  if(true == bStatus)
  {
    bStatus = imuSelfTest();

    if (true == bStatus)
    {
      // We need a reset, because we want to revert the settings configured during the self-test.
      bStatus = softResetImu();
    }

    if(true == bStatus)
    {
      bStatus = setAccelerometerDynamicRange(skeAccelDynamicRange_);
    }

    if(true == bStatus)
    {
      bStatus = setGyroscopeDynamicRange(skeGyroDynamicRange_);
    }

    if(true == bStatus)
    {
      bStatus = setImuSampleRate();
    }

    if(true == bStatus)
    {
      bStatus = setImuFilter();
    }

    if(true == bStatus)
    {
      bStatus = enableAllImuAxes();
    }

    if(true == bStatus)
    {
      bStatus = enableBypass();
    }
  }

  // Initialize the barometer
  if(true == bStatus)
  {
    bStatus = softResetBaro();
  }

  if(true == bStatus)
  {
    // Check ID of the barometer
    bStatus = getChipIDBaro(uBaroId);

    if(true == bStatus)
    {
      if (uBaroId != ICM20789_PRESSURE_ID)
      {
        bStatus = false;
      }
    }

    if (true == bStatus)
    {
      bStatus = getOTPBaro();
    }
  }

  bIsInitialized_ = bStatus;
}

bool CIcm20789Driver::IsInitialized()
{
  return bIsInitialized_;
}

void CIcm20789Driver::PollPressureSensor()
{
  SBarometerMeasurement oOutput;

  if (true == IsInitialized())
  {
    float fTemp = 0.0F, fPress = 0.0F;
    uint64_t uTimestamp = GetMicroseconds();

    if(true == getTemperatureAndPressure(fTemp, fPress))
    {
      oOutput.fTemperature_ = fTemp;
      oOutput.fPressure_ = fPress;
      oOutput.uTimestampUs_ = uTimestamp;
      oOutput.uValid_ = BoolToUint(true);
    }
    else
    {
      oOutput.uValid_ = BoolToUint(false);
    }

    // Trigger a measurement again
    setLowNoiseModeBaro();
  }
  else
  {
    oOutput.uValid_ = BoolToUint(false);
  }

  if (EIcmIds::eIcm1 == keSensorId_)
  {
    CRte::GetInstance().oPortIcm20789BaroInput1_.Write(oOutput);
  }
  else if (EIcmIds::eIcm2 == keSensorId_)
  {
    CRte::GetInstance().oPortIcm20789BaroInput2_.Write(oOutput);
  }
  else
  {
    // do nothing
  }
}

void CIcm20789Driver::PollInertialSensor()
{
  bool bStatus;
  uint8_t auCtrl[ICM20789_IMU_BYTES_TO_POLL] = {0};
  int16_t iAccelX{0}, iAccelY{0}, iAccelZ{0};
  int16_t iGyroX{0}, iGyroY{0}, iGyroZ{0};
  int16_t iTemp{0};
  SImuMeasurement oOutput;

  if (true == IsInitialized())
  {
    uint64_t uTimestamp = GetMicroseconds();

    bStatus = readFromImuRegisters(ICM20789_IMU_REG_ACCEL_XOUT_H, auCtrl, 14);

    if (true == bStatus)
    {
      iAccelX = (auCtrl[0] << 8) | auCtrl[1];
      iAccelY = (auCtrl[2] << 8) | auCtrl[3];
      iAccelZ = (auCtrl[4] << 8) | auCtrl[5];

      iTemp = (auCtrl[6] << 8) | auCtrl[7];

      iGyroX = (auCtrl[8] << 8) | auCtrl[9];
      iGyroY = (auCtrl[10] << 8) | auCtrl[11];
      iGyroZ = (auCtrl[12] << 8) | auCtrl[13];

      oOutput.fTemperature_ = (((static_cast<float>(iTemp)) - skfTempOffset_) / skfTempScale_) + skfTempOffset_;
      oOutput.fSpecificForceX_ = -skfAccelRawToMetersPerSecondSquared_ * static_cast<float>(iAccelX);
      oOutput.fSpecificForceY_ = skfAccelRawToMetersPerSecondSquared_ * static_cast<float>(iAccelY);
      oOutput.fSpecificForceZ_ = -skfAccelRawToMetersPerSecondSquared_ * static_cast<float>(iAccelZ);

      oOutput.fAngularRateX_ = -skfGyroRawToRadiansPerSecond_ * static_cast<float>(iGyroX);
      oOutput.fAngularRateY_ = skfGyroRawToRadiansPerSecond_ * static_cast<float>(iGyroY);
      oOutput.fAngularRateZ_ = -skfGyroRawToRadiansPerSecond_ * static_cast<float>(iGyroZ);

      oOutput.uTimestampUs_ = uTimestamp;
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

  if (EIcmIds::eIcm1 == keSensorId_)
  {
    CRte::GetInstance().oPortIcm20789ImuInput1_.Write(oOutput);
  }
  else if (EIcmIds::eIcm2 == keSensorId_)
  {
    CRte::GetInstance().oPortIcm20789ImuInput2_.Write(oOutput);
  }
  else
  {
    // do nothing
  }
}

bool CIcm20789Driver::getChipIDBaro(uint8_t& urId)
{
  bool bStatus;
  uint8_t auCtrl[3];

  auCtrl[0] = 0xEF;
  auCtrl[1] = 0xC8;

  bStatus = i2cWrite(ICM20789_ADDRESS_PRESS, auCtrl, 2);

  if (true == bStatus)
  {
    bStatus = i2cRead(ICM20789_ADDRESS_PRESS, auCtrl, 3);
  }

  if (true == bStatus)
  {
    uint8_t uCrc = crc8(auCtrl, 2);
    bStatus = (uCrc == auCtrl[2]);
  }

  if (true == bStatus)
  {
    urId = auCtrl[1] & static_cast<uint16_t>(~0xC0); // 0xC0 = 11000000b
  }

  return bStatus;
}

bool CIcm20789Driver::softResetBaro()
{
  bool bStatus;
  uint8_t auCtrl[2];

  auCtrl[0] = 0x80;
  auCtrl[1] = 0x5D;

  bStatus = i2cWrite(ICM20789_ADDRESS_PRESS, auCtrl, 2);

  HAL_Delay(10);

  return bStatus;
}

bool CIcm20789Driver::getOTPBaro()
{
  uint8_t auDataWrite[10];
  uint8_t auDataRead[10] = {0};
  uint8_t uIndex;
  bool bStatus;

  // OTP Read mode
  auDataWrite[0] = 0xC5;
  auDataWrite[1] = 0x95;
  auDataWrite[2] = 0x00;
  auDataWrite[3] = 0x66;
  auDataWrite[4] = 0x9C;

  bStatus = i2cWrite(ICM20789_ADDRESS_PRESS, auDataWrite, 5);

  // Read OTP values
  if (true == bStatus)
  {
    for (uIndex = 0U; uIndex < 4U; uIndex++)
    {
      auDataWrite[0] = 0xC7;
      auDataWrite[1] = 0xF7;

      bStatus = i2cWrite(ICM20789_ADDRESS_PRESS, auDataWrite, 2);

      if (true == bStatus)
      {
        bStatus = i2cReadOtp(ICM20789_ADDRESS_PRESS, auDataRead, 3);
      }

      if (true == bStatus)
      {
        uint8_t uCrc = crc8(auDataRead, 2);
        bStatus = (uCrc == auDataRead[2]);
      }

      if (false == bStatus)
      {
        break;
      }

      oCalibParam_.afSensorConstants_[uIndex] = static_cast<float>((auDataRead[0] << 8) | auDataRead[1]);
    }
  }

  if (true == bStatus)
  {
    oCalibParam_.afPaCalib_[0] = 45000.0F;
    oCalibParam_.afPaCalib_[1] = 80000.0F;
    oCalibParam_.afPaCalib_[2] = 105000.0F;
    oCalibParam_.fLutLower_ = 3.5F * static_cast<float>(1 << 20);
    oCalibParam_.fLutUpper_ = 11.5F * static_cast<float>(1 << 20);
    oCalibParam_.fQuadrFactor_ = 1.0F / 16777216.0F;
    oCalibParam_.fOffstFactor_ = 2048.0F;
  }

  return bStatus;
}

bool CIcm20789Driver::getTemperatureAndPressure(float& frTemperature, float& frPressure)
{
  bool bStatus;
  uint8_t auData[9] = {0};

  bStatus = i2cRead(ICM20789_ADDRESS_PRESS, auData, 9);

  if((auData[2] != crc8(&auData[0], 2)) ||
     (auData[5] != crc8(&auData[3], 2)) ||
     (auData[8] != crc8(&auData[6], 2)))
  {
    bStatus = false;
  }

  if (true == bStatus)
  {
    uint32_t uTraw = (static_cast<uint32_t>(auData[0]) << 8) | static_cast<uint32_t>(auData[1]);
    uint32_t uPraw = (static_cast<uint32_t>(auData[3]) << 16) | (static_cast<uint32_t>(auData[4]) << 8) | static_cast<uint32_t>(auData[6]);

    processDataBaro(uPraw, uTraw, frPressure, frTemperature);
  }

  return bStatus;
}

void CIcm20789Driver::processDataBaro(uint32_t uPressData, uint32_t uTempData, float& frPressure, float& frTemperature)
{
  float fT;
  float afIn[3];
  float fA, fB, fC;

  fT = static_cast<float>(static_cast<int32_t>(uTempData) - 32768);

  afIn[0] = oCalibParam_.fLutLower_ + oCalibParam_.afSensorConstants_[0] * fT * fT * oCalibParam_.fQuadrFactor_;
  afIn[1] = oCalibParam_.fOffstFactor_ * oCalibParam_.afSensorConstants_[3] + oCalibParam_.afSensorConstants_[1] * fT * fT * oCalibParam_.fQuadrFactor_;
  afIn[2] = oCalibParam_.fLutUpper_ + oCalibParam_.afSensorConstants_[2] * fT * fT * oCalibParam_.fQuadrFactor_;

  fC = (afIn[0] * afIn[1] * (oCalibParam_.afPaCalib_[0] - oCalibParam_.afPaCalib_[1]) + afIn[1] * afIn[2] * (oCalibParam_.afPaCalib_[1] - oCalibParam_.afPaCalib_[2]) +
       afIn[2] * afIn[0] * (oCalibParam_.afPaCalib_[2] - oCalibParam_.afPaCalib_[0])) / (afIn[2] * (oCalibParam_.afPaCalib_[0] - oCalibParam_.afPaCalib_[1]) +
       afIn[0] * (oCalibParam_.afPaCalib_[1] - oCalibParam_.afPaCalib_[2]) + afIn[1] * (oCalibParam_.afPaCalib_[2] - oCalibParam_.afPaCalib_[0]));

  fA = (oCalibParam_.afPaCalib_[0] * afIn[0] - oCalibParam_.afPaCalib_[1] * afIn[1] - (oCalibParam_.afPaCalib_[1] - oCalibParam_.afPaCalib_[0]) * fC) / (afIn[0] - afIn[1]);
  fB = (oCalibParam_.afPaCalib_[0] - fA) * (afIn[0] + fC);

  frPressure = fA + fB / (fC + static_cast<float>(uPressData));
  frTemperature = -45.0F + 175.0F / 65536.0F * static_cast<float>(uTempData);
}

bool CIcm20789Driver::setLowNoiseModeBaro()
{
  uint8_t uSts = 0;
  uint8_t auCtrl[2];

  auCtrl[0] = 0x70;
  auCtrl[1] = 0xDF;

  uSts = i2cWrite(ICM20789_ADDRESS_PRESS, auCtrl, 2);

  return uSts;
}

bool CIcm20789Driver::getImuChipId(uint8_t& urId)
{
  return readFromImuRegisters(ICM20789_IMU_REG_WHO_AM_I, &urId, 1);
}

bool CIcm20789Driver::softResetImu()
{
  bool bStatus;

  // Request reset
  bStatus = writeImuRegister(ICM20789_IMU_REG_PWR_MGMT1, 0x80);

  // We need to wake the chip up, because it sleeps by default.
  // Additionally we configure clock source to be auto-selected.
  if (true == bStatus)
  {
    HAL_Delay(100);
    bStatus = writeImuRegister(ICM20789_IMU_REG_PWR_MGMT1, ICM20789_PWR_MGMT1_VALUE);
  }

  return bStatus;
}

bool CIcm20789Driver::setAccelerometerDynamicRange(EAccelDynamicRange eAccelDynamicRange)
{
  uint8_t uRegisterValue;

  bool bStatus = readFromImuRegisters(ICM20789_IMU_REG_ACCEL_CONFIG, &uRegisterValue, 1);

  if (true == bStatus)
  {
    uint8_t uMask = 0xE7; // 0xE7 = 11100111b
    uRegisterValue &= uMask;
    uRegisterValue |= ((static_cast<uint8_t>(eAccelDynamicRange) << 3) & (~uMask));
    bStatus = writeImuRegister(ICM20789_IMU_REG_ACCEL_CONFIG, uRegisterValue);
  }

  return bStatus;
}

bool CIcm20789Driver::setGyroscopeDynamicRange(EGyroDynamicRange eGyroDynamicRange)
{
  uint8_t uRegisterValue;

  bool bStatus = readFromImuRegisters(ICM20789_IMU_REG_GYRO_CONFIG, &uRegisterValue, 1);

  if (true == bStatus)
  {
    uint8_t uMask = 0xE7; // 0xE7 = 11100111b
    uRegisterValue &= uMask;
    uRegisterValue |= ((static_cast<uint8_t>(eGyroDynamicRange) << 3) & (~uMask));
    bStatus = writeImuRegister(ICM20789_IMU_REG_GYRO_CONFIG, uRegisterValue);
  }

  return bStatus;
}

bool CIcm20789Driver::enableAllImuAxes()
{
  bool bStatus = writeImuRegister(ICM20789_IMU_REG_PWR_MGMT2, ICM20789_PWR_MGMT2_VALUE);
  HAL_Delay(ICM20789_GYRO_ENGINE_UP_TIME);
  return bStatus;
}

bool CIcm20789Driver::setImuSampleRate()
{
  return writeImuRegister(ICM20789_IMU_REG_SMPLRT_DIV, ICM20789_SMPLRT_DIV_VALUE);
}

bool CIcm20789Driver::setImuFilter()
{
  // We need to configure some bits in two registers
  uint8_t uRegisterValue;

  bool bStatus = readFromImuRegisters(ICM20789_IMU_REG_CONFIG, &uRegisterValue, 1);

  if (true == bStatus)
  {
    // We make sure that bits 0 to 2 are zeros
    uRegisterValue &= static_cast<uint8_t>(~0x07);
    bStatus = writeImuRegister(ICM20789_IMU_REG_CONFIG, uRegisterValue);
  }

  if (true == bStatus)
  {
    bStatus = readFromImuRegisters(ICM20789_IMU_REG_GYRO_CONFIG, &uRegisterValue, 1);
  }

  if (true == bStatus)
  {
    // We set to zero the bits 0 and 1
    uRegisterValue &= static_cast<uint8_t>(~0x03);
    bStatus = writeImuRegister(ICM20789_IMU_REG_GYRO_CONFIG, uRegisterValue);
  }

  return bStatus;
}

bool CIcm20789Driver::i2cRead(uint8_t uIicAddress, uint8_t* upData, uint8_t uLen)
{
  HAL_StatusTypeDef uSts = HAL_OK;
  uSts = HAL_I2C_Master_Receive(opkI2CHandle_, ((uIicAddress << 1)), upData, uLen, ICM20789_TIMEOUT);
  return (uSts == HAL_OK ? true : false);
}

bool CIcm20789Driver::i2cReadOtp(uint8_t uIicAddress, uint8_t* upData, uint8_t uLen)
{
  HAL_StatusTypeDef uSts = HAL_OK;
  uSts = HAL_I2C_Master_Receive(opkI2CHandle_, ((uIicAddress << 1)), upData, uLen, ICM20789_TIMEOUT_OTP);
  return (uSts == HAL_OK ? true : false);
}

bool CIcm20789Driver::i2cWrite(uint8_t uIicAddress, uint8_t* upData, uint8_t uLen)
{
  HAL_StatusTypeDef uSts = HAL_OK;
  uSts = HAL_I2C_Master_Transmit(opkI2CHandle_, (uIicAddress << 1), upData, uLen, ICM20789_TIMEOUT);
  return (uSts == HAL_OK ? true : false);
}

bool CIcm20789Driver::readFromImuRegisters(uint8_t uReg, uint8_t* upData, uint8_t uLen)
{
   bool bStatus = i2cWrite(ICM20789_ADDRESS_IMU, &uReg, 1);

   if (true == bStatus)
   {
     bStatus = i2cRead(ICM20789_ADDRESS_IMU, upData, uLen);
   }

   return bStatus;
}

bool CIcm20789Driver:: writeImuRegister(uint8_t uReg, uint8_t uData)
{
  uint8_t auData[2] = { uReg, uData };
  return i2cWrite(ICM20789_ADDRESS_IMU, auData, 2);
}

bool CIcm20789Driver::imuSelfTest()
{
  bool bStatus;
  int aiDataAccel[3] = {0};
  int aiDataAccelSelfTest[3] = {0};
  int aiDataGyro[3] = {0};
  int aiDataGyroSelfTest[3] = {0};

  bStatus = enableAllImuAxes();

  if(true == bStatus)
  {
    bStatus = collectTestStatistics(false, aiDataGyro, aiDataAccel);
  }

  if(true == bStatus)
  {
    bStatus = collectTestStatistics(true, aiDataGyroSelfTest, aiDataAccelSelfTest);
  }

  if (true == bStatus)
  {
    bool bAccelerometerResult = checkAccelSelfTest(aiDataAccel, aiDataAccelSelfTest);
    bool bGyroscopeResult = checkGyroSelfTest(aiDataGyro, aiDataGyroSelfTest);
    bStatus = bAccelerometerResult && bGyroscopeResult;
  }

  return bStatus;
}

bool CIcm20789Driver::enableBypass()
{
  uint8_t uRegData;
  bool bStatus;

  // Set bit 1 of the INT_PIN_CFG to 1
  bStatus = readFromImuRegisters(ICM20789_IMU_REG_INT_PIN_CFG, &uRegData, 1);

  if (true == bStatus)
  {
    uRegData |= static_cast<uint8_t>(0x02);
    bStatus = writeImuRegister(ICM20789_IMU_REG_INT_PIN_CFG, uRegData);
  }

  // Set bit 4 of the user control register to 0
  if (true == bStatus)
  {
    bStatus = readFromImuRegisters(ICM20789_IMU_REG_USER_CTRL, &uRegData, 1);
  }

  if (true == bStatus)
  {
    uRegData &= (~static_cast<uint8_t>(0x10));
    bStatus = writeImuRegister(ICM20789_IMU_REG_USER_CTRL, uRegData);
  }

  return bStatus;
}

uint8_t CIcm20789Driver::crc8(uint8_t *upData, uint8_t uLen)
{
  uint8_t uCrc = 0xFF;
  unsigned int uIndex;

  while ((uLen--) > 0)
  {
    uCrc ^= *upData;
    upData++;

    for (uIndex = 0; uIndex < 8; uIndex++)
    {
      uCrc = uCrc & 0x80 ? (uCrc << 1) ^ 0x31 : uCrc << 1;
    }
  }

  return uCrc;
}

bool CIcm20789Driver::collectTestStatistics(bool bWithSelfTestFlag, int* ipGyroResult, int* ipAccelResult)
{
  bool bStatus;
  uint8_t auData[ICM20789_IMU_BYTES_TO_POLL];
  int iPacket, iAxis;
  const int kiPacketSize = 12;
  int iCounter = 0;
  int iFifoCount = 0;
  int iPacketCount = 0;
  int iIndex = 0;

  bStatus = writeImuRegister(ICM20789_IMU_REG_PWR_MGMT2, 0x00);

  /* clear signal path */
  if(true == bStatus)
  {
    HAL_Delay(ICM20789_GYRO_ENGINE_UP_TIME);
    bStatus = writeImuRegister(ICM20789_IMU_REG_USER_CTRL, 0x01);
  }

  /* disable interrupt */
  if(true == bStatus)
  {
    HAL_Delay(ICM20789_GYRO_ENGINE_UP_TIME);
    bStatus = writeImuRegister(ICM20789_IMU_REG_INT_ENABLE, 0x00);
  }

  /* disable the sensor output to FIFO */
  if(true == bStatus)
  {
    bStatus = writeImuRegister(ICM20789_IMU_REG_FIFO_EN, 0x00);
  }

  /* disable fifo reading */
  if(true == bStatus)
  {
    bStatus = writeImuRegister(ICM20789_IMU_REG_USER_CTRL, 0x00);
  }

  /* setup parameters */
  if(true == bStatus)
  {
    bStatus = writeImuRegister(ICM20789_IMU_REG_CONFIG, 0x02);
  }

  if(true == bStatus)
  {
    bStatus = writeImuRegister(ICM20789_IMU_REG_LP_CONFIG, 0x00);
  }

  /* config accel LPF register */
  if(true == bStatus)
  {
    bStatus = writeImuRegister(ICM20789_IMU_REG_ACCEL_CONFIG_2, 0x02);
  }

  if(true == bStatus)
  {
    bStatus = writeImuRegister(ICM20789_IMU_REG_SMPLRT_DIV, 0x00);
  }

  uint8_t uSelfTestFlags = 0U;

  if (true == bWithSelfTestFlag)
  {
    uSelfTestFlags = ICM20789_BIT_XG_ST | ICM20789_BIT_YG_ST | ICM20789_BIT_ZG_ST;
  }

  if(true == bStatus)
  {
    HAL_Delay(ICM20789_GYRO_ENGINE_UP_TIME);
    bStatus = writeImuRegister(ICM20789_IMU_REG_GYRO_CONFIG, uSelfTestFlags);
  }

  if(true == bStatus)
  {
    bStatus = writeImuRegister(ICM20789_IMU_REG_ACCEL_CONFIG, uSelfTestFlags);
    HAL_Delay(ICM20789_GYRO_ENGINE_UP_TIME);
  }

  if (true == bStatus)
  {
    while(iCounter < 200)
    {
      /* clear FIFO */
      bStatus = writeImuRegister(ICM20789_IMU_REG_USER_CTRL, ICM20789_BIT_FIFO_RST);

      /* enable FIFO reading */
      if(true == bStatus)
      {
        bStatus = writeImuRegister(ICM20789_IMU_REG_USER_CTRL, ICM20789_BIT_FIFO_EN);
      }

      if(true == bStatus)
      {
        bStatus = writeImuRegister(ICM20789_IMU_REG_FIFO_EN, 0x78);
      }

      if(true == bStatus)
      {
        bStatus = writeImuRegister(ICM20789_IMU_REG_PWR_MGMT1, ICM20789_PWR_MGMT1_VALUE);
        HAL_Delay(5);
      }

      if(true == bStatus)
      {
        bStatus = writeImuRegister(ICM20789_IMU_REG_FIFO_EN, 0x00); // stop FIFO
      }

      if(true == bStatus)
      {
        bStatus = readFromImuRegisters(ICM20789_IMU_REG_FIFO_COUNTH, auData, 2);
      }

      if (true == bStatus)
      {
        iFifoCount = (auData[0] << 8) | auData[1];
        iPacketCount = iFifoCount / kiPacketSize;

        iPacket = 0;

        while((iPacket < iPacketCount) && (iCounter < 200))
        {
          short aiVals[3];

          if(true == bStatus)
          {
            bStatus = readFromImuRegisters(ICM20789_IMU_REG_FIFO_R_W, auData, static_cast<uint8_t>(kiPacketSize));
          }
          else
          {
            break;
          }

          iIndex = 0;

          for(iAxis = 0; iAxis < 3; iAxis++)
          {
            aiVals[iAxis] = (auData[iIndex + 2 * iAxis] << 8) | auData[iIndex + 2 * iAxis + 1];
            ipAccelResult[iAxis] += aiVals[iAxis];
          }

          iIndex += ICM20789_BYTES_PER_IMU_SENSOR;

          for(iAxis = 0; iAxis < 3; iAxis++)
          {
            aiVals[iAxis] = (auData[iIndex + 2 * iAxis] << 8) | auData[iIndex + 2 * iAxis + 1];
            ipGyroResult[iAxis] += aiVals[iAxis];
          }

          iCounter++;
          iPacket++;
        }
      }

      if (false == bStatus)
      {
        break;
      }
    }
  }

  if (true == bStatus)
  {
    for(iAxis = 0; iAxis < 3; iAxis++)
    {
      ipAccelResult[iAxis] = ipAccelResult[iAxis] / iCounter;
      ipAccelResult[iAxis] *= ICM20789_SELF_TEST_PRECISION;
    }

    for(iAxis = 0; iAxis < 3; iAxis++)
    {
      ipGyroResult[iAxis] = ipGyroResult[iAxis] / iCounter;
      ipGyroResult[iAxis] *= ICM20789_SELF_TEST_PRECISION;
    }
  }

  return bStatus;
}

bool CIcm20789Driver::checkAccelSelfTest(int* ipMeanNormalTestValues, int* ipMeanSelfTestValues)
{
  bool bResult = true;
  uint8_t auRegs[3];
  bool bOtpValueZero = false;
  int aiSelfTestShiftProd[3], aiSelfTestShiftCust[3], aiSelfTestShiftRatio[3], iIndex;
  bool bStatus;

  bStatus = readFromImuRegisters(ICM20789_IMU_REG_ACCEL_X_ST, auRegs, 3);

  if(true == bStatus)
  {
    for(iIndex = 0; iIndex < 3; iIndex++)
    {
      if(auRegs[iIndex] != 0)
      {
        aiSelfTestShiftProd[iIndex] = kauSelfTestEquation[auRegs[iIndex] - 1];
      }
      else
      {
        aiSelfTestShiftProd[iIndex] = 0;
        bOtpValueZero = true;
      }
    }

    if(false == bOtpValueZero)
    {
      /* Self Test Pass/Fail Criteria A */
      for(iIndex = 0; iIndex < 3; iIndex++)
      {
        aiSelfTestShiftCust[iIndex] = ipMeanSelfTestValues[iIndex] - ipMeanNormalTestValues[iIndex];
        aiSelfTestShiftRatio[iIndex] = abs(aiSelfTestShiftCust[iIndex] / aiSelfTestShiftProd[iIndex] - ICM20789_SELF_TEST_PRECISION);

        if(aiSelfTestShiftRatio[iIndex] > ICM20789_ACCEL_SELF_TEST_SHIFT_DELTA)
        {
          bResult = false;
        }
      }
    }
    else
    {
      /* Self Test Pass/Fail Criteria B */
      for(iIndex = 0; iIndex < 3; iIndex++)
      {
        aiSelfTestShiftCust[iIndex] = abs(ipMeanSelfTestValues[iIndex] - ipMeanNormalTestValues[iIndex]);

        if((aiSelfTestShiftCust[iIndex] < ACCEL_SELF_TEST_AL_MIN) || (aiSelfTestShiftCust[iIndex] > ACCEL_SELF_TEST_AL_MAX))
        {
          bResult = false;
        }
      }
    }
  }
  else
  {
    bResult = false;
  }

  return bResult;
}

bool CIcm20789Driver::checkGyroSelfTest(int* ipMeanNormalTestValues, int* ipMeanSelfTestValues)
{
  bool bResult = true;
  uint8_t auRegs[3];
  bool bOtpValueZero = false;
  int aiSelfTestShiftProd[3], aiSelfTestShiftCust[3], iIndex;
  bool bStatus;

  bStatus = readFromImuRegisters(ICM20789_IMU_REG_GYRO_X_ST, auRegs, 3);

  if(true == bStatus)
  {
    for(iIndex = 0; iIndex < 3; iIndex++)
    {
      if(auRegs[iIndex] != 0)
      {
        aiSelfTestShiftProd[iIndex] = kauSelfTestEquation[auRegs[iIndex] - 1];
      }
      else
      {
        aiSelfTestShiftProd[iIndex] = 0;
        bOtpValueZero = true;
      }
    }

    for (iIndex = 0; iIndex < 3; iIndex++)
    {
      aiSelfTestShiftCust[iIndex] = ipMeanSelfTestValues[iIndex] - ipMeanNormalTestValues[iIndex];

      if (false == bOtpValueZero)
      {
        /* Self Test Pass/Fail Criteria A */
        if (aiSelfTestShiftCust[iIndex] < ICM20789_GYRO_SELF_TEST_SHIFT_DELTA * aiSelfTestShiftProd[iIndex])
        {
          bResult = false;
        }
      }
      else
      {
        /* Self Test Pass/Fail Criteria B */
        if (aiSelfTestShiftCust[iIndex] < ICM20789_GYRO_SELF_TEST_AL * ICM20789_SELF_TEST_GYRO_SENS * ICM20789_SELF_TEST_PRECISION)
        {
          bResult = false;
        }
      }
    }

    if (true == bResult)
    {
      /* Self Test Pass/Fail Criteria C */
      for (iIndex = 0; iIndex < 3; iIndex++)
      {
        if (abs(ipMeanNormalTestValues[iIndex]) > ICM20789_GYRO_OFFSET_MAX * (ICM20789_SELF_TEST_SCALE / ICM20789_SELF_TEST_GYRO_FS_DPS) * ICM20789_SELF_TEST_PRECISION)
        {
          bResult = false;
          break;
        }
      }
    }
  }
  else
  {
    bResult = false;
  }

  return bResult;
}

I2C_HandleTypeDef* CIcm20789Driver::getI2CHandle(EIcmIds eIcmChipId)
{
  I2C_HandleTypeDef* opHandle;

  if (EIcmIds::eIcm1 == eIcmChipId)
  {
    opHandle = &hi2c1;
  }
  else if (EIcmIds::eIcm2 == eIcmChipId)
  {
    opHandle = &hi2c3;
  }
  else
  {
    opHandle = nullptr;
  }

  return opHandle;
}
