/**
 * @file CScha63TDriver.cpp
 * @brief Implementation of the SCHA63T driver software component.
 * @author Fedor Baklanov
 * @date 07 Mai 2022
 */

#include "main.h"
#include "GetMicroseconds.h"
#include "tim.h"
#include "cmsis_os.h"
#include "CRte.h"
#include "UintToBool.h"
#include <cstdio> // We include this file because of sprintf().

extern SPI_HandleTypeDef hspi1; // The global SPI object handle is generated by the STM32CubeIDE. See spi.c.

extern "C" void PollScha63T()
{
  CScha63TDriver::GetInstance().PollSensor();
}

CScha63TDriver& CScha63TDriver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CScha63TDriver soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CScha63TDriver::Init()
{
  bool bStatusDue = false;
  bool bStatusUno = false;

  bIsInitialized_ = true;

  // HW Reset (reset via SPI)
  spiWriteReadUno(SPI_FRAME_WRITE_RESET);
  spiWriteReadDue(SPI_FRAME_WRITE_REG_BANK_0);  // Make sure we are in bank 0, otherwise SPI reset is not available.
  spiWriteReadDue(SPI_FRAME_WRITE_RESET);

  // Wait 25 ms for the non-volatile memory (NVM) Read
  HAL_Delay(25);

  // Read UNO asic serial number
  spiWriteReadUno(SPI_FRAME_READ_TRC_2);
  uint16_t uTrc2 = frameToUint16(spiWriteReadUno(SPI_FRAME_READ_TRC_0));
  uint16_t uTrc0 = frameToUint16(spiWriteReadUno(SPI_FRAME_READ_TRC_1));
  uint16_t uTrc1 = frameToUint16(spiWriteReadUno(SPI_FRAME_READ_TRC_1));

  // Build serial number string
  uint16_t uId1 = (uTrc2 >> 8) & 0x0F;
  uint16_t uId0 = uTrc0 & 0xFFFF;
  uint16_t uId2 = uTrc1 & 0xFFFF;
  sprintf(acSerialNumber_, "%05d%01x%04X", uId2, uId1, uId0);

  // Activate DUE asic test mode to be able to read cross-axis
  // compensation values from DUE NVM (see p.2.6.1 of Murata datasheet)
  spiWriteReadDue(SPI_FRAME_WRITE_MODE_ASM_010);          //Write to Mode Register word = 'RRRRRRRR RR010RRRb'
  spiWriteReadDue(SPI_FRAME_READ_MODE);                   //Write to Mode Register ReadMode word = '00000000 00000000b'
  spiWriteReadDue(SPI_FRAME_WRITE_MODE_ASM_001);          //Write to Mode Register word = 'RRRRRRRR RR001RRRb'
  spiWriteReadDue(SPI_FRAME_READ_MODE);                   //Write to Mode Register ReadMode word = '00000000 00000000b'
  spiWriteReadDue(SPI_FRAME_WRITE_MODE_ASM_100);          //Write to Mode Register word = 'RRRRRRRR RR100RRRb'
  spiWriteReadDue(SPI_FRAME_READ_MODE);                   //Write to Mode Register ReadMode word = '00000000 00000000b'
  uint32_t uResp = spiWriteReadDue(SPI_FRAME_READ_MODE);  //Read Mode Register content

  // Check if device is correctly set to test mode
  if ((frameToUint16(uResp) & 0x07) == 0x07)
  {
    // Read cross-axis compensation values
    spiWriteReadDue(0xFC00051A);  //Write data 0x05 to address 0x1F
    spiWriteReadDue(0x2C0000CB);
    uint32_t uCxxCxy = spiWriteReadDue(0x4C00009B);
    uint32_t uCxzCyx = spiWriteReadDue(0x50000089);
    uint32_t uCyyCyz = spiWriteReadDue(0x5400008F);
    uint32_t uCzxCzy = spiWriteReadDue(0x58000085);
    uint32_t uCzzBxx = spiWriteReadDue(0x5C000083);
    uint32_t uBxyBxz = spiWriteReadDue(0x600000A1);
    uint32_t uByxByy = spiWriteReadDue(0x6C0000AB);
    uint32_t uByzBzx = spiWriteReadDue(0x700000B9);
    uint32_t uBzyBzz = spiWriteReadDue(0x700000B9);

    oCompensationParameters_.fCxx_ = static_cast<float>(lowerInt8FromFrame(uCxxCxy)) / 4096.0F + 1.0F;
    oCompensationParameters_.fCxy_ = static_cast<float>(upperInt8FromFrame(uCxxCxy)) / 4096.0F;
    oCompensationParameters_.fCxz_ = static_cast<float>(lowerInt8FromFrame(uCxzCyx)) / 4096.0F;
    oCompensationParameters_.fCyx_ = static_cast<float>(upperInt8FromFrame(uCxzCyx)) / 4096.0F;
    oCompensationParameters_.fCyy_ = static_cast<float>(lowerInt8FromFrame(uCyyCyz)) / 4096.0F + 1.0F;
    oCompensationParameters_.fCyz_ = static_cast<float>(upperInt8FromFrame(uCyyCyz)) / 4096.0F;
    oCompensationParameters_.fCzx_ = static_cast<float>(lowerInt8FromFrame(uCzxCzy)) / 4096.0F;
    oCompensationParameters_.fCzy_ = static_cast<float>(upperInt8FromFrame(uCzxCzy)) / 4096.0F;
    oCompensationParameters_.fCzz_ = static_cast<float>(lowerInt8FromFrame(uCzzBxx)) / 4096.0F + 1.0F;
    oCompensationParameters_.fBxx_ = static_cast<float>(upperInt8FromFrame(uCzzBxx)) / 4096.0F + 1.0F;
    oCompensationParameters_.fBxy_ = static_cast<float>(lowerInt8FromFrame(uBxyBxz)) / 4096.0F;
    oCompensationParameters_.fBxz_ = static_cast<float>(upperInt8FromFrame(uBxyBxz)) / 4096.0F;
    oCompensationParameters_.fByx_ = static_cast<float>(lowerInt8FromFrame(uByxByy)) / 4096.0F;
    oCompensationParameters_.fByy_ = static_cast<float>(upperInt8FromFrame(uByxByy)) / 4096.0F + 1.0F;
    oCompensationParameters_.fByz_ = static_cast<float>(lowerInt8FromFrame(uByzBzx)) / 4096.0F;
    oCompensationParameters_.fBzx_ = static_cast<float>(upperInt8FromFrame(uByzBzx)) / 4096.0F;
    oCompensationParameters_.fBzy_ = static_cast<float>(lowerInt8FromFrame(uBzyBzz)) / 4096.0F;
    oCompensationParameters_.fBzz_ = static_cast<float>(upperInt8FromFrame(uBzyBzz)) / 4096.0F + 1.0F;
  }
  else
  {
    bIsInitialized_ = false;
  }

  if (true == bIsInitialized_)
  {
    // HW Reset to get DUE asic out from test mode (reset via SPI).
    spiWriteReadDue(SPI_FRAME_WRITE_REG_BANK_0);         // Return to bank 0 to make SPI reset command available.
    spiWriteReadDue(SPI_FRAME_WRITE_RESET);              // Reset DUE after reading cross-axis registers


    // Start UNO & DUE
    HAL_Delay(25);                                       // Wait 25ms for the non-volatile memory (NVM) Read

    // DUE asic initial startup
    spiWriteReadUno(SPI_FRAME_WRITE_OP_MODE_NORMAL);     // Set UNO operation mode on
    spiWriteReadDue(SPI_FRAME_WRITE_OP_MODE_NORMAL);     // Set DUE operation mode on twice
    spiWriteReadDue(SPI_FRAME_WRITE_OP_MODE_NORMAL);

    HAL_Delay(70);                                       // Wait minimum 70ms (includes UNO 50ms 'SPI accessible' wait)

    spiWriteReadUno(SPI_FRAME_WRITE_FILTER_300HZ_RATE);  // Select UNO 300Hz filter for RATE
    spiWriteReadUno(SPI_FRAME_WRITE_FILTER_300HZ_ACC);   // Select UNO 300Hz filter for ACC

    // Restart DUE
    spiWriteReadDue(SPI_FRAME_WRITE_RESET);              // Reset DUE again

    HAL_Delay(25);                                       // Wait 25 ms for the NVM read

    spiWriteReadDue(SPI_FRAME_WRITE_OP_MODE_NORMAL);     // Set DUE operation mode
    spiWriteReadDue(SPI_FRAME_WRITE_OP_MODE_NORMAL);     // DUE operation mode must be set twice

    HAL_Delay(1);                                        // Wait 1 ms for SPI to be accessible

    spiWriteReadDue(SPI_FRAME_WRITE_FILTER_300HZ_RATE);  // Select DUE 300Hz filter for RATE

    for (int iAttempt = 0; iAttempt < skiMaxAttemptsToConfigure_; iAttempt++)
    {
      // Wait 405 ms (Gyro and ACC start up)
      HAL_Delay(405);

      // Set EOI=1 (End of initialization command)
      spiWriteReadUno(SPI_FRAME_WRITE_EOI_BIT);              // Set EOI bit for UNO
      spiWriteReadDue(SPI_FRAME_WRITE_EOI_BIT);              // Set EOI bit for DUE

      // Check initialization status from the summary status registers
      bStatusUno = pollUnoStatus();
      bStatusDue = pollDueStatus();

      // Check if restart is needed
      if (((false == bStatusUno) || (false == bStatusDue)) && (iAttempt < (skiMaxAttemptsToConfigure_ - 1)))
      {
        spiWriteReadUno(SPI_FRAME_WRITE_RESET);
        spiWriteReadDue(SPI_FRAME_WRITE_RESET);

        HAL_Delay(25);                                       // Wait 25 ms for the NVM read

        spiWriteReadUno(SPI_FRAME_WRITE_OP_MODE_NORMAL);     // Set UNO operation mode on
        spiWriteReadDue(SPI_FRAME_WRITE_OP_MODE_NORMAL);     // Set DUE operation mode on twice
        spiWriteReadDue(SPI_FRAME_WRITE_OP_MODE_NORMAL);

        HAL_Delay(50);                                       // Wait 50ms before communicating with UNO

        spiWriteReadUno(SPI_FRAME_WRITE_FILTER_300HZ_RATE);  // Select UNO 300Hz filter for RATE
        spiWriteReadUno(SPI_FRAME_WRITE_FILTER_300HZ_ACC);   // Select UNO 300Hz filter for ACC
        spiWriteReadDue(SPI_FRAME_WRITE_FILTER_300HZ_RATE);  // Select DUE 300Hz filter for RATE

        HAL_Delay(45);                                       // Adjust restart duration to 500 ms
      }
      else
      {
        break;
      }
    }

    if ((false == bStatusDue) || (false == bStatusUno))
    {
      bIsInitialized_ = false;
    }
  }

  if (true == bIsInitialized_)
  {
    // We start the TIM7 in interrupt mode. The handle is declared in the tim.h, generated by the STM32CubeIDE.
    HAL_TIM_Base_Start_IT(&htim7);
  }
}

bool CScha63TDriver::IsInitialized()
{
  return bIsInitialized_;
}

void CScha63TDriver::PollSensor()
{
  static uint32_t uCounter = 0U;

  if (true == bIsInitialized_)
  {
    uint32_t auFrames[eFrameCount] = { 0U };

    ++uCounter;

    __disable_irq();

    if (1U == uCounter)
    {
      oLatestDataset_.uTimestampFirstUs_ = GetMicroseconds();
    }
    else if (SAMPLE_COUNT == uCounter)
    {
      oLatestDataset_.uTimestampLastUs_ = GetMicroseconds();
    }
    else
    {
      // Do nothing
    }

    spiWriteReadDue(SPI_FRAME_READ_GYRO_Y);
    auFrames[eAngularRateY] = spiWriteReadDue(SPI_FRAME_READ_GYRO_Z);
    auFrames[eAngularRateZ] =  spiWriteReadDue(SPI_FRAME_READ_TEMP);
    auFrames[eTemperatureDue] = spiWriteReadDue(SPI_FRAME_READ_TEMP);

    spiWriteReadUno(SPI_FRAME_READ_GYRO_X);
    auFrames[eAngularRateX] = spiWriteReadUno(SPI_FRAME_READ_ACC_X);
    auFrames[eSpecificForceX] = spiWriteReadUno(SPI_FRAME_READ_ACC_Y);
    auFrames[eSpecificForceY] = spiWriteReadUno(SPI_FRAME_READ_ACC_Z);
    auFrames[eSpecificForceZ] = spiWriteReadUno(SPI_FRAME_READ_TEMP);
    auFrames[eTemperatureUno] = spiWriteReadUno(SPI_FRAME_READ_TEMP);

    __enable_irq();

    bErrorFlags_ = checkRsErrorInFrames(auFrames, eFrameCount);

    SScha63TMeasurement oMeasurement = { 0 };

    if (false == bErrorFlags_)
    {
      oMeasurement.iSpecificForceX_ = frameToInt16(auFrames[eSpecificForceX]);
      oMeasurement.iSpecificForceY_ = frameToInt16(auFrames[eSpecificForceY]);
      oMeasurement.iSpecificForceZ_ = frameToInt16(auFrames[eSpecificForceZ]);
      oMeasurement.iAngularRateX_ = frameToInt16(auFrames[eAngularRateX]);
      oMeasurement.iAngularRateY_ = frameToInt16(auFrames[eAngularRateY]);
      oMeasurement.iAngularRateZ_ = frameToInt16(auFrames[eAngularRateZ]);
      oMeasurement.iTemperatureUno_ = frameToInt16(auFrames[eTemperatureUno]);
      oMeasurement.iTemperatureDue_ = frameToInt16(auFrames[eTemperatureDue]);
      oMeasurement.uValid_ = 1U;
    }
    else
    {
      __disable_irq();

      spiWriteReadUno(SPI_FRAME_READ_SUMMARY_STATUS);
      oStatusUno_.uSummaryStatus_ = frameToUint16(spiWriteReadUno(SPI_FRAME_READ_RATE_STATUS_1));
      oStatusUno_.uRateStatus_ = frameToUint16(spiWriteReadUno(SPI_FRAME_READ_ACC_STATUS_1));
      oStatusUno_.uAccelerometerStatus_ = frameToUint16(spiWriteReadUno(SPI_FRAME_READ_COMMON_STATUS_1));
      oStatusUno_.uCommonStatus1_ = frameToUint16(spiWriteReadUno(SPI_FRAME_READ_COMMON_STATUS_2));
      oStatusUno_.uCommonStatus2_ = frameToUint16(spiWriteReadUno(SPI_FRAME_READ_COMMON_STATUS_2));

      spiWriteReadDue(SPI_FRAME_READ_SUMMARY_STATUS);
      oStatusDue_.uSummaryStatus_ = frameToUint16(spiWriteReadDue(SPI_FRAME_READ_RATE_STATUS_1));
      oStatusDue_.uRateStatus1_ = frameToUint16(spiWriteReadDue(SPI_FRAME_READ_RATE_STATUS_2));
      oStatusDue_.uRateStatus2_ = frameToUint16(spiWriteReadDue(SPI_FRAME_READ_COMMON_STATUS_1));
      oStatusDue_.uCommonStatus1_ = frameToUint16(spiWriteReadDue(SPI_FRAME_READ_COMMON_STATUS_2));
      oStatusDue_.uCommonStatus2_ = frameToUint16(spiWriteReadDue(SPI_FRAME_READ_COMMON_STATUS_2));

      __enable_irq();
    }

    oLatestDataset_.aoMeasurements_[uCounter - 1U] = oMeasurement;

    if (SAMPLE_COUNT == uCounter)
    {
      __disable_irq();

      if (false == bDatasetAvailable_)
      {
        bDatasetAvailable_ = true;
        oOutputDataset_ = oLatestDataset_;
      }

      __enable_irq();

      uCounter = 0U;
    }
  }
}

void CScha63TDriver::ConvertRawDataset()
{
  SImuDataScha63T oImuData;
  SScha63TDataset_t oDataset;
  CRte& orRte = CRte::GetInstance();

  __disable_irq();

  bool bDatasetAvailable = bDatasetAvailable_;

  if (true == bDatasetAvailable)
  {
    oDataset = oOutputDataset_;
    bDatasetAvailable_ = false;
  }

  __enable_irq();

  if ((true == bIsInitialized_) && (true == bDatasetAvailable))
  {
    unsigned iMeasurementCount = 0U;

    for (unsigned iIndex = 0; iIndex < SAMPLE_COUNT; ++iIndex)
    {
      if (1U == oDataset.aoMeasurements_[iIndex].uValid_)
      {
        ++iMeasurementCount;
        oImuData.fSpecificForceX_ += static_cast<float>(oDataset.aoMeasurements_[iIndex].iSpecificForceX_);
        oImuData.fSpecificForceY_ += static_cast<float>(oDataset.aoMeasurements_[iIndex].iSpecificForceY_);
        oImuData.fSpecificForceZ_ += static_cast<float>(oDataset.aoMeasurements_[iIndex].iSpecificForceZ_);
        oImuData.fAngularRateX_ += static_cast<float>(oDataset.aoMeasurements_[iIndex].iAngularRateX_);
        oImuData.fAngularRateY_ += static_cast<float>(oDataset.aoMeasurements_[iIndex].iAngularRateY_);
        oImuData.fAngularRateZ_ += static_cast<float>(oDataset.aoMeasurements_[iIndex].iAngularRateZ_);
      }
    }

    if (iMeasurementCount > (SAMPLE_COUNT - 3U))
    {
      float fTmp = skfGravity_ / (skfAccelerometerSensitivity_ * static_cast<float>(iMeasurementCount));
      oImuData.fSpecificForceX_ *= fTmp;
      oImuData.fSpecificForceY_ *= fTmp;
      oImuData.fSpecificForceZ_ *= fTmp;

      fTmp = skfDegreesToRadians_ / (skfGyroscopeSensitivity_ * static_cast<float>(iMeasurementCount));
      oImuData.fAngularRateX_ *= fTmp;
      oImuData.fAngularRateY_ *= fTmp;
      oImuData.fAngularRateZ_ *= fTmp;

      SImuDataScha63T oImuDataTmp;
      const SCompensationParameters& korParams = oCompensationParameters_;

      oImuDataTmp.fSpecificForceX_ = (korParams.fBxx_ * oImuData.fSpecificForceX_) + (korParams.fBxy_ * oImuData.fSpecificForceY_) + (korParams.fBxz_ * oImuData.fSpecificForceZ_);
      oImuDataTmp.fSpecificForceY_ = (korParams.fByx_ * oImuData.fSpecificForceX_) + (korParams.fByy_ * oImuData.fSpecificForceY_) + (korParams.fByz_ * oImuData.fSpecificForceZ_);
      oImuDataTmp.fSpecificForceZ_ = (korParams.fBzx_ * oImuData.fSpecificForceX_) + (korParams.fBzy_ * oImuData.fSpecificForceY_) + (korParams.fBzz_ * oImuData.fSpecificForceZ_);
      oImuDataTmp.fAngularRateX_ = (korParams.fCxx_ * oImuData.fAngularRateX_) + (korParams.fCxy_ * oImuData.fAngularRateY_) + (korParams.fCxz_ * oImuData.fAngularRateZ_);
      oImuDataTmp.fAngularRateY_ = (korParams.fCyx_ * oImuData.fAngularRateX_) + (korParams.fCyy_ * oImuData.fAngularRateY_) + (korParams.fCyz_ * oImuData.fAngularRateZ_);
      oImuDataTmp.fAngularRateZ_ = (korParams.fCzx_ * oImuData.fAngularRateX_) + (korParams.fCzy_ * oImuData.fAngularRateY_) + (korParams.fCzz_ * oImuData.fAngularRateZ_);

      /* Sensor-frame to body-frame output conversion:
       *
       *        /  0  1  0 \
       * C_bs = | -1  0  0 |
       *        \  0  0 -1 /
      */

      oImuData.fSpecificForceX_ = oImuDataTmp.fSpecificForceY_;
      oImuData.fSpecificForceY_ = -oImuDataTmp.fSpecificForceX_;
      oImuData.fSpecificForceZ_ = -oImuDataTmp.fSpecificForceZ_;
      oImuData.fAngularRateX_ = oImuDataTmp.fAngularRateY_;
      oImuData.fAngularRateY_ = -oImuDataTmp.fAngularRateX_;
      oImuData.fAngularRateZ_ = -oImuDataTmp.fAngularRateZ_;

      oImuData.uTimestampUs_ = (oDataset.uTimestampLastUs_ >> 1) + (oDataset.uTimestampFirstUs_ >> 1);
      oImuData.uValid_ = BoolToUint(true);
    }
    else
    {
      oImuData = SImuDataScha63T();
    }
  }

  orRte.oScha63TDriverPort_.Write(oImuData);
}

bool CScha63TDriver::pollUnoStatus()
{
  uint32_t uResp;
  bool bUnoOk;

  // Read summary status two times (first time may show incorrectly FAIL after start-up)
  spiWriteReadUno(SPI_FRAME_READ_SUMMARY_STATUS);
  spiWriteReadUno(SPI_FRAME_READ_SUMMARY_STATUS);

  HAL_Delay(3);

  uResp = spiWriteReadUno(SPI_FRAME_READ_SUMMARY_STATUS);

  bUnoOk = ((checkRsError(uResp) == true) ? false : true);

  return bUnoOk;
}

bool CScha63TDriver::pollDueStatus()
{
  uint32_t uResp;
  bool bDueOk;

  // Read summary status two times (first time may show incorrectly FAIL after start-up)
  spiWriteReadDue(SPI_FRAME_READ_SUMMARY_STATUS);
  spiWriteReadDue(SPI_FRAME_READ_SUMMARY_STATUS);

  HAL_Delay(3) ;

  uResp = spiWriteReadDue(SPI_FRAME_READ_SUMMARY_STATUS);

  bDueOk = ((checkRsError(uResp) == true) ? false : true);

  return bDueOk;
}

inline bool CScha63TDriver::checkRsError(uint32_t uFrame)
{
  return ((((uFrame >> 24) & 0x03) != 1U) ? true : false);
}

bool CScha63TDriver::checkRsErrorInFrames(uint32_t* upFrames, unsigned uFrameCount)
{
  bool bRetVal = false;

  for (unsigned uIndex = 0U; uIndex < uFrameCount; ++uIndex)
  {
      if (true == checkRsError(upFrames[uIndex]))
      {
        bRetVal = true;
        break;
      }
  }

  return bRetVal;
}

uint32_t CScha63TDriver::spiWriteReadUno(uint32_t uDataOut)
{
  return spiWriteRead(uDataOut, MURATA_CSB_UNO_GPIO_Port, MURATA_CSB_UNO_Pin);
}

uint32_t CScha63TDriver::spiWriteReadDue(uint32_t uDataOut)
{
  return spiWriteRead(uDataOut, MURATA_CSB_DUE_GPIO_Port, MURATA_CSB_DUE_Pin);
}

uint32_t CScha63TDriver::spiWriteRead(uint32_t uDataOut, GPIO_TypeDef* opGpioX, uint16_t uGpioPin)
{
  uint32_t uResp = 0;
  uint8_t uDataByte[4];
  uint8_t uRespByte[4];
  HAL_StatusTypeDef eMurataSpiStatus;
  HAL_SPI_StateTypeDef eMurataSpiState;

  HAL_GPIO_WritePin(opGpioX, uGpioPin, GPIO_PIN_RESET); // Set CS active

  // We send data in 8-bits. As STM32 is little-endian, we need to change the byte order.
  uDataByte[3] = static_cast<uint8_t>(uDataOut & 0x000000FF);
  uDataByte[2] = static_cast<uint8_t>((uDataOut >> 8) & 0x000000FF);
  uDataByte[1] = static_cast<uint8_t>((uDataOut >> 16) & 0x000000FF);
  uDataByte[0] = static_cast<uint8_t>((uDataOut >> 24) & 0x000000FF);

  do
  {
    eMurataSpiState = HAL_SPI_GetState(&hspi1);
  }
  while(eMurataSpiState != HAL_SPI_STATE_READY);

  eMurataSpiStatus = HAL_SPI_TransmitReceive(&hspi1, &uDataByte[0], &uRespByte[0], 4, 1);

  if(eMurataSpiStatus == HAL_OK)
  {
    uResp = (static_cast<uint32_t>(uRespByte[0]) << 24) & 0xFF000000;
    uResp |= (static_cast<uint32_t>(uRespByte[1]) << 16) & 0x00FF0000;
    uResp |= (static_cast<uint32_t>(uRespByte[2]) << 8) & 0x0000FF00;
    uResp |= static_cast<uint32_t>(uRespByte[3]) & 0x000000FF;
  }

  HAL_GPIO_WritePin(opGpioX, uGpioPin, GPIO_PIN_SET); // Set CS non-active

  return uResp;
}

inline int16_t CScha63TDriver::frameToInt16(uint32_t uFrame)
{
  return static_cast<int16_t>((uFrame >> 8) & 0xFFFF);
}

inline uint16_t CScha63TDriver::frameToUint16(uint32_t uFrame)
{
  return static_cast<uint16_t>((uFrame >> 8) & 0xFFFF);
}

inline int8_t CScha63TDriver::lowerInt8FromFrame(uint32_t uFrame)
{
  return static_cast<int8_t>((uFrame >> 8) & 0xFF);
}

inline int8_t CScha63TDriver::upperInt8FromFrame(uint32_t uFrame)
{
  return static_cast<int8_t>((uFrame >> 16) & 0xFF);
}
