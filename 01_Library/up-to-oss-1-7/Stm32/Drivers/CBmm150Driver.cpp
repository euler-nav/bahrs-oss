/**
 * @file CBmm150Driver.cpp
 * @brief Implementation of the BMM150 driver software component
 * @author Fedor Baklanov
 * @date 08 August 2022
 */

#include "CBmm150Driver.h"
#include "CRte.h"
#include "main.h"
#include "GetMicroseconds.h"
#include "stm32f4xx_hal_fmpi2c.h"
#include "stm32f4xx_hal.h"
#include "RteTypes.h"

extern FMPI2C_HandleTypeDef hfmpi2c1;
extern I2C_HandleTypeDef hi2c2;

CBmm150Driver::CBmm150Driver(EBmmIds eBmmId) :
    keSensorId_(eBmmId)
{
  if (EBmmIds::eInvalid == keSensorId_)
  {
    // We must not instantiate an invalid driver.
    assert(false);
  }
}

CBmm150Driver& CBmm150Driver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CBmm150Driver soDriverInstance1(EBmmIds::eBmm1), soDriverInstance2(EBmmIds::eBmm2);

  assert(uInstanceIndex < CBmm150Driver::skuInstanceCount_);

  CBmm150Driver* opInstance;

  if (0U == uInstanceIndex)
  {
    opInstance = &soDriverInstance1;
  }
  else if (1U == uInstanceIndex)
  {
    opInstance = &soDriverInstance2;
  }
  else
  {
    assert(false);
  }

  return *opInstance;
}

bool CBmm150Driver::readFromAddr(uint8_t uRegAddr, uint8_t* upRegData, uint8_t uSize)
{
  HAL_StatusTypeDef eSts;
  bool bStatus = false;
  uint16_t uAddrByte = static_cast<uint16_t>(BMM150_DEF_I2C_ADDR) << 1;

  switch(keSensorId_)
  {
    case EBmmIds::eBmm1:
      /* Send slave addr and register addr to be read */
      eSts = HAL_FMPI2C_Master_Transmit(&hfmpi2c1, uAddrByte, &uRegAddr, 1, BMM150_TIMEOUT_1MS);

      if(eSts == HAL_OK)
      {
        /* Read register data */
        eSts = HAL_FMPI2C_Master_Receive(&hfmpi2c1, uAddrByte, upRegData, uSize, BMM150_TIMEOUT_1MS);

        if (eSts == HAL_OK)
        {
          bStatus = true;
        }
      }

      break;
    case EBmmIds::eBmm2:
      /* Send slave addr and register addr to be read */
      eSts = HAL_I2C_Master_Transmit(&hi2c2, uAddrByte, &uRegAddr, 1, BMM150_TIMEOUT_1MS);

      if(eSts == HAL_OK)
      {
        /* Read register data */
        eSts = HAL_I2C_Master_Receive(&hi2c2, uAddrByte, upRegData, uSize, BMM150_TIMEOUT_1MS);

        if (eSts == HAL_OK)
        {
          bStatus = true;
        }
      }

      break;
    default:
      // This function shall not be called if the sensor ID is invalid.
      assert(false);
      break;
  }

  return bStatus;
}

bool CBmm150Driver::writeRegister(uint8_t uRegAddr, uint8_t* upRegData)
{
  HAL_StatusTypeDef eSts = HAL_OK;
  uint8_t auRegInfo[2];
  uint16_t uAddrByte = static_cast<uint16_t>(BMM150_DEF_I2C_ADDR) << 1;

  auRegInfo[0] = uRegAddr;
  auRegInfo[1] = *upRegData;

  /* Send slave addr, register addr and data to be written */
  switch(keSensorId_)
  {
    case EBmmIds::eBmm1:
      eSts = HAL_FMPI2C_Master_Transmit(&hfmpi2c1, uAddrByte, auRegInfo, 2, BMM150_TIMEOUT_1MS);
      break;
    case EBmmIds::eBmm2:
      eSts = HAL_I2C_Master_Transmit(&hi2c2, uAddrByte, auRegInfo, 2, BMM150_TIMEOUT_1MS);
      break;
    default:
      // This function shall not be called if the sensor ID is invalid.
      assert(false);
      break;
  }

  return (HAL_OK == eSts) ? true : false;
}

bool CBmm150Driver::setNormalOperationMode(void)
{
  uint8_t uRegData = 0;
  uint32_t uOpMode = 0x0; //Normal Operation mode
  bool bStatus = false;

  /* Read 0x4C register */
  if(readFromAddr(0x4c, &uRegData, 1))
  {
    /* Set op_mode */
    uRegData = ((uRegData & ~(0x3 << 1)) | ((uOpMode & 0x3) << 1));
    if(writeRegister(0x4c, &uRegData))
    {
      bStatus = true;
    }
  }

  return bStatus;
}

bool CBmm150Driver::setPresetConfig(uint8_t uXYRep, uint8_t uZRep, uint8_t uODR)
{
  uint8_t uRegData;
  bool bStatus = false;

  /* Read the 0x4C register */
  if(readFromAddr(0x4C, &uRegData, 1))
  {
    /* set ODR */
    uRegData = ((uRegData & ~(0x7 << 3)) | ((uODR & 0x7) << 3));
    if(writeRegister(0x4C, &uRegData))
    {
      /* set xy-axis repetition */
      if(writeRegister(0x51, &uXYRep))
      {
        /* set z-axis repetition */
        if(writeRegister(0x52, &uZRep))
        {
          bStatus = true;
        }
      }
    }
  }

  return bStatus;
}

bool CBmm150Driver::readTrimRegisters(void)
{
  uint8_t auTrimX1Y1[2] = { 0 };
  uint8_t auTrimXYZData[4] = { 0 };
  uint8_t auTrimXY1XY2[10] = { 0 };
  uint16_t uTempMSB = 0;
  bool bStatus = false;

  /* Trim register value is read */
  if(readFromAddr(0x5D, auTrimX1Y1, 2))
  {
    if(readFromAddr(0x62, auTrimXYZData, 4))
    {
      if(readFromAddr(0x68, auTrimXY1XY2, 10))
      {
        oTrimRegData_.iDigX1_ = static_cast<int8_t>(auTrimX1Y1[0]);
        oTrimRegData_.iDigY1_ = static_cast<int8_t>(auTrimX1Y1[1]);
        oTrimRegData_.iDigX2_ = static_cast<int8_t>(auTrimXYZData[2]);
        oTrimRegData_.iDigY2_ = static_cast<int8_t>(auTrimXYZData[3]);
        uTempMSB = static_cast<uint16_t>(auTrimXY1XY2[3]) << 8;
        oTrimRegData_.uDigZ1_ = static_cast<uint16_t>(uTempMSB | auTrimXY1XY2[2]);
        uTempMSB = static_cast<uint16_t>(auTrimXY1XY2[1]) << 8;
        oTrimRegData_.iDigZ2_ = static_cast<int16_t>(uTempMSB | auTrimXY1XY2[0]);
        uTempMSB = static_cast<uint16_t>(auTrimXY1XY2[7]) << 8;
        oTrimRegData_.iDigZ3_ = static_cast<int16_t>(uTempMSB | auTrimXY1XY2[6]);
        uTempMSB = static_cast<uint16_t>(auTrimXYZData[1]) << 8;
        oTrimRegData_.iDigZ4_ = static_cast<int16_t>(uTempMSB | auTrimXYZData[0]);
        oTrimRegData_.uDigXY1_ = auTrimXY1XY2[9];
        oTrimRegData_.iDigXY2_ = static_cast<int8_t>(auTrimXY1XY2[8]);
        uTempMSB = static_cast<uint16_t>(auTrimXY1XY2[5] & 0x7F) << 8;
        oTrimRegData_.uDigXYZ1_ = static_cast<uint16_t>(uTempMSB | auTrimXY1XY2[4]);
        bStatus = true;
      }
    }
  }

  return bStatus;
}

int16_t CBmm150Driver::compensateX(int16_t iMagDataX, uint16_t uDataRHall)
{
  int16_t iRetVal;
  uint16_t uProcessCompX0 = 0;
  int32_t iProcessCompX1;
  uint16_t uProcessCompX2;
  int32_t iProcessCompX3;
  int32_t iProcessCompX4;
  int32_t iProcessCompX5;
  int32_t iProcessCompX6;
  int32_t iProcessCompX7;
  int32_t iProcessCompX8;
  int32_t iProcessCompX9;
  int32_t iProcessCompX10;

  /* Overflow condition check */
  if(iMagDataX != skiOvrflwAdcValXYaxisFlip_)
  {
    if(uDataRHall != 0)
    {
      /* Availability of valid data */
      uProcessCompX0 = uDataRHall;
    }
    else if(oTrimRegData_.uDigXYZ1_ != 0)
    {
      uProcessCompX0 = oTrimRegData_.uDigXYZ1_;
    }
    else
    {
      uProcessCompX0 = 0;
    }

    if(uProcessCompX0 != 0)
    {
      /* Processing compensation equations */
      iProcessCompX1 = (static_cast<int32_t>(oTrimRegData_.uDigXYZ1_) * 16384);
      uProcessCompX2 = (static_cast<uint16_t>(iProcessCompX1 / uProcessCompX0)) - (static_cast<uint16_t>(0x4000));
      iRetVal = static_cast<int16_t>(uProcessCompX2);
      iProcessCompX3 = ((static_cast<int32_t>(iRetVal)) * (static_cast<int32_t>(iRetVal)));
      iProcessCompX4 = ((static_cast<int32_t>(oTrimRegData_.iDigXY2_)) * (iProcessCompX3 / 128));
      iProcessCompX5 = static_cast<int32_t>(static_cast<int16_t>(oTrimRegData_.uDigXY1_) * 128);
      iProcessCompX6 = (static_cast<int32_t>(iRetVal)) * iProcessCompX5;
      iProcessCompX7 = (((iProcessCompX4 + iProcessCompX6) / 512) + (static_cast<int32_t>(0x100000)));
      iProcessCompX8 = (static_cast<int32_t>((static_cast<int16_t>(oTrimRegData_.iDigX2_)) + (static_cast<int16_t>(0xA0))));
      iProcessCompX9 = ((iProcessCompX7 * iProcessCompX8) / 4096);
      iProcessCompX10 = static_cast<int32_t>(iMagDataX) * iProcessCompX9;
      iRetVal = (static_cast<int16_t>(iProcessCompX10 / 8192));
      iRetVal = (iRetVal + (static_cast<int16_t>(oTrimRegData_.iDigX1_) * 8)) / 16;
    }
    else
    {
      iRetVal = skiOvrflwOutput_;
    }
  }
  else
  {
    /* Overflow condition */
    iRetVal = skiOvrflwOutput_;
  }

  return iRetVal;
}

int16_t CBmm150Driver::compensateY(int16_t iMagDataY, uint16_t uDataRHall)
{
  int16_t iRetVal;
  uint16_t uProcessCompY0 = 0;
  int32_t iProcessCompY1;
  uint16_t uProcessCompY2;
  int32_t iProcessCompY3;
  int32_t iProcessCompY4;
  int32_t iProcessCompY5;
  int32_t iProcessCompY6;
  int32_t iProcessCompY7;
  int32_t iProcessCompY8;
  int32_t iProcessCompY9;

  /* Overflow condition check */
  if(iMagDataY != skiOvrflwAdcValXYaxisFlip_)
  {
    if(uDataRHall != 0)
    {
      /* Availability of valid data */
      uProcessCompY0 = uDataRHall;
    }
    else if(oTrimRegData_.uDigXYZ1_ != 0)
    {
      uProcessCompY0 = oTrimRegData_.uDigXYZ1_;
    }
    else
    {
      uProcessCompY0 = 0;
    }

    if(uProcessCompY0 != 0)
    {
      /* Processing compensation equations */
      iProcessCompY1 = ((static_cast<int32_t>(oTrimRegData_.uDigXYZ1_)) * 16384) / uProcessCompY0;
      uProcessCompY2 = (static_cast<uint16_t>(iProcessCompY1)) - (static_cast<uint16_t>(0x4000));
      iRetVal = static_cast<int16_t>(uProcessCompY2);
      iProcessCompY3 = static_cast<int32_t>(iRetVal) * static_cast<int32_t>(iRetVal);
      iProcessCompY4 = static_cast<int32_t>(oTrimRegData_.iDigXY2_) * (iProcessCompY3 / 128);
      iProcessCompY5 = static_cast<int32_t>(static_cast<int16_t>(oTrimRegData_.uDigXY1_) * 128);
      iProcessCompY6 = ((iProcessCompY4 + (static_cast<int32_t>(iRetVal) * iProcessCompY5)) / 512);
      iProcessCompY7 = static_cast<int32_t>(static_cast<int16_t>(oTrimRegData_.iDigY2_) + static_cast<int16_t>(0xA0));
      iProcessCompY8 = (((iProcessCompY6 + static_cast<int32_t>(0x100000)) * iProcessCompY7) / 4096);
      iProcessCompY9 = (static_cast<int32_t>(iMagDataY) * iProcessCompY8);
      iRetVal = static_cast<int16_t>(iProcessCompY9 / 8192);
      iRetVal = (iRetVal + (static_cast<int16_t>(oTrimRegData_.iDigY1_) * 8)) / 16;
    }
    else
    {
      iRetVal = skiOvrflwOutput_;
    }
  }
  else
  {
    /* Overflow condition */
    iRetVal = skiOvrflwOutput_;
  }

  return iRetVal;
}

int16_t CBmm150Driver::compensateZ(int16_t iMagDataZ, uint16_t uDataRHall)
{
  int32_t iRetVal;
  int16_t iProcessCompZ0;
  int32_t iProcessCompZ1;
  int32_t iProcessCompZ2;
  int32_t iProcessCompZ3;
  int16_t iProcessCompZ4;

  if(iMagDataZ != skiOvrflwAdcValZaxisHall_)
  {
    if((oTrimRegData_.iDigZ2_ != 0) &&
       (oTrimRegData_.uDigZ1_ != 0) &&
       (uDataRHall != 0) && (oTrimRegData_.uDigXYZ1_ != 0))
    {
      /* Processing compensation equations */
      iProcessCompZ0 = static_cast<int16_t>(uDataRHall) - static_cast<int16_t>(oTrimRegData_.uDigXYZ1_);
      iProcessCompZ1 = ((static_cast<int32_t>(oTrimRegData_.iDigZ3_)) * (static_cast<int32_t>(iProcessCompZ0))) / 4;
      iProcessCompZ2 = (static_cast<int32_t>(iMagDataZ - oTrimRegData_.iDigZ4_) * 32768);
      iProcessCompZ3 = (static_cast<int32_t>(oTrimRegData_.uDigZ1_)) * ((static_cast<int16_t>(uDataRHall)) * 2);
      iProcessCompZ4 = static_cast<int16_t>((iProcessCompZ3 + (32768)) / 65536);
      iRetVal = ((iProcessCompZ2 - iProcessCompZ1) / (oTrimRegData_.iDigZ2_ + iProcessCompZ4));

      /* Saturate result to +/- 2 micro-tesla */
      if(iRetVal > skiPosSaturationZaxis_)
      {
        iRetVal = skiPosSaturationZaxis_;
      }
      else if(iRetVal < skiNegSaturationZaxis_)
      {
        iRetVal = skiNegSaturationZaxis_;
      }

      /* Conversion of LSB to micro-tesla */
      iRetVal = iRetVal / 16;
    }
    else
    {
      iRetVal = skiOvrflwOutput_;
    }
  }
  else
  {
    /* Overflow condition */
    iRetVal = skiOvrflwOutput_;
  }

  return (static_cast<int16_t>(iRetVal));
}

void CBmm150Driver::Init(void)
{
  uint8_t uChipId = 0;
  uint8_t uData = 0x01;
  bool bStatus;

  bStatus = IsInitialized();
  if(false == bStatus)
  {
    /* Device is not initialized, Initalize it here */

    /* On POR sensor is in suspend state */

    /* Set power control bit */
    bStatus = writeRegister(BMM150_REG_PWR_CTRL, &uData);
    if(true == bStatus)
    {
      /* Delay 3ms */
      HAL_Delay(3);

      /* At this line sensor is in sleep state */

      /* Read sensor chip id */
      bStatus = readFromAddr(BMM150_REG_CHIPID, &uChipId, 1);
    }

    if(true == bStatus)
    {
      /* Is it BMM150 sensor? */
      bStatus = (uChipId == BMM150_DEF_CHIP_ID_VAL) ? true : false;
    }

    if(true == bStatus)
    {
      /* Function to update trim values */
      bStatus = readTrimRegisters();
    }

    if (true == bStatus)
    {
      /* Set Normal operating mode */
      bStatus = setNormalOperationMode();
    }

    if (true == bStatus)
    {
      /* Set Enhanced Preset sensor configurations
       * params as below :
       * 15 - xy_rep
       * 27 - z_rep
       * 0 - 10Hz ODR
       */
      bStatus = setPresetConfig(15, 27, 0);
    }

    if(true == bStatus)
    {
      uSensorStatus_ = 1U;
    }
  }

  return;
}

/**
 * @brief Read Magnetic data from BMM150 Sensor
 */
void CBmm150Driver::Bmm150ReadMagData(void)
{
  uint8_t auRawData[8];
  int16_t iRawX;
  int16_t iRawY;
  int16_t iRawZ;
  int16_t iRawR;
  SMagneticData oMagData;
  int16_t iTmpMsb;

  if(IsInitialized())
  {
    oMagData.uTimeStampUs_ = GetMicroseconds();

    if (readFromAddr(0x42, auRawData, 8))
    {
      iTmpMsb = ((static_cast<int16_t>(static_cast<int8_t>(auRawData[1]))) << 5);
      iRawX = static_cast<int16_t>(iTmpMsb | ((auRawData[0] & 0xF8) >> 3));

      iTmpMsb = ((static_cast<int16_t>(static_cast<int8_t>(auRawData[3]))) << 5);
      iRawY = static_cast<int16_t>(iTmpMsb | ((auRawData[2] & 0xF8) >> 3));

      iTmpMsb = ((static_cast<int16_t>(static_cast<int8_t>(auRawData[5]))) << 7);
      iRawZ = static_cast<int16_t>(iTmpMsb | ((auRawData[4] & 0xFE) >> 1));

      iTmpMsb = ((static_cast<int16_t>(static_cast<int8_t>(auRawData[7]))) << 6);
      iRawR = static_cast<int16_t>(iTmpMsb | ((auRawData[6] & 0xFC) >> 2));

      float fValX = compensateX(iRawX, iRawR);
      float fValY = compensateY(iRawY, iRawR);
      float fValZ = compensateZ(iRawZ, iRawR);

      /* Convert uTesla to Gauss, 1uT => 0.01 Gauss  */
      oMagData.fXAxisVal_ = (-fValY) * 0.01;
      oMagData.fYAxisVal_ = (-fValX) * 0.01;
      oMagData.fZAxisVal_ = (-fValZ) * 0.01;

      oMagData.bDataIsValid_ = true;
    }
    else
    {
      oMagData.bDataIsValid_ = false;
    }
  }
  else
  {
    oMagData.bDataIsValid_ = false;
  }

  if (EBmmIds::eBmm1 == keSensorId_)
  {
    CRte::GetInstance().oPortBmm150Input1_.Write(oMagData);
  }
  else if (EBmmIds::eBmm2 == keSensorId_)
  {
    CRte::GetInstance().oPortBmm150Input2_.Write(oMagData);
  }
  else
  {
    // Do nothing
  }
}

/**
 * @brief Check BMM150 sensor is initialized
 * @return true if initialized else return false
 */
bool CBmm150Driver::IsInitialized(void)
{
  return (uSensorStatus_ == 1U) ? true : false;
}
