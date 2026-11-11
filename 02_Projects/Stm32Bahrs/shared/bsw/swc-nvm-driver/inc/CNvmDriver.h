/** ---- IMPORTANT ----
 *      
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * Date: 08.08.2026 at 20:08:24
*/
#ifndef C_NVM_DRIVER_H
#define C_NVM_DRIVER_H

#include <stdint.h>
#include <memory.h>

#include "main.h"
#include "stm32f4xx_hal_fmpi2c.h"
#include "General/CSoftwareComponentBase.h"

extern FMPI2C_HandleTypeDef hfmpi2c1;

class CNvmDriver : public CSoftwareComponent<CNvmDriver, 0U>
{
  friend class CSoftwareComponent<CNvmDriver, 0U>;
  FORBID_CLASS_COPY_AND_MOVE(CNvmDriver)
  DECLARE_MANDATORY_APIS(CNvmDriver)

public:

  static constexpr uint16_t skuMemorySize_ { 8000U };
  static constexpr uint8_t skuPageSize_ { 32U };
  static constexpr uint16_t skuVersionMajor_ { 2U };
  static constexpr uint16_t skuVersionMinor_ { 0U };

#pragma pack(push, 4)


  struct SNmvFormatVersion
  {
    uint32_t uMajor_;
    uint32_t uMinor_;
    uint32_t uCrc_;
  };

  struct SEulerAnglesDeviceToVehicle
  {
    float fRoll_;
    float fPitch_;
    float fYaw_;
    uint32_t uCrc_;
  };

  struct SMagnetometerCalibrationParameters1
  {
    float afHardIronOffset_[3];
    float afSoftIronDistortion_[9];
    uint32_t uCrc_;
  };

  struct SMagnetometerCalibrationParameters2
  {
    float afHardIronOffset_[3];
    float afSoftIronDistortion_[9];
    uint32_t uCrc_;
  };

  struct SMagnetometerCalibrationParameters3
  {
    float afHardIronOffset_[3];
    float afSoftIronDistortion_[9];
    uint32_t uCrc_;
  };

  struct SCanConfigurationParameters
  {
    uint32_t uNodeID_;
    uint32_t uRedundantChannelID_;
    uint32_t uServiceChannelID_;
    uint32_t uUpdateRateFrame300_;
    uint32_t uUpdateRateFrame301_;
    uint32_t uUpdateRateFrame302_;
    uint32_t uUpdateRateFrame303_;
    uint32_t uUpdateRateFrame304_;
    uint32_t uUpdateRateFrame305_;
    uint32_t uUpdateRateFrame311_;
    uint32_t uUpdateRateFrame312_;
    uint32_t uUpdateRateFrame314_;
    uint32_t uUpdateRateFrame1069_;
    uint32_t uUpdateRateFrame322_;
    uint32_t uUpdateRateFrame326_;
    uint32_t uCrc_;
  };

  struct SPressureCompensationParameters
  {
    float afPressureOffset_[3];
    uint32_t uCrc_;
  };


  struct SNvmData
  {
    SNmvFormatVersion oNmvFormatVersion;
    SEulerAnglesDeviceToVehicle oEulerAnglesDeviceToVehicle;
    SMagnetometerCalibrationParameters1 oMagnetometerCalibrationParameters1;
    SMagnetometerCalibrationParameters2 oMagnetometerCalibrationParameters2;
    SMagnetometerCalibrationParameters3 oMagnetometerCalibrationParameters3;
    SCanConfigurationParameters oCanConfigurationParameters;
    SPressureCompensationParameters oPressureCompensationParameters;
  };

  static_assert(sizeof(SNvmData) <= skuMemorySize_);

#pragma pack(pop)

  static constexpr uint16_t skuBlockOffsetNmvFormatVersion = 0U;
  static constexpr uint16_t skuBlockOffsetEulerAnglesDeviceToVehicle = skuBlockOffsetNmvFormatVersion + sizeof(SNmvFormatVersion);
  static constexpr uint16_t skuBlockOffsetMagnetometerCalibrationParameters1 = skuBlockOffsetEulerAnglesDeviceToVehicle + sizeof(SEulerAnglesDeviceToVehicle);
  static constexpr uint16_t skuBlockOffsetMagnetometerCalibrationParameters2 = skuBlockOffsetMagnetometerCalibrationParameters1 + sizeof(SMagnetometerCalibrationParameters1);
  static constexpr uint16_t skuBlockOffsetMagnetometerCalibrationParameters3 = skuBlockOffsetMagnetometerCalibrationParameters2 + sizeof(SMagnetometerCalibrationParameters2);
  static constexpr uint16_t skuBlockOffsetCanConfigurationParameters = skuBlockOffsetMagnetometerCalibrationParameters3 + sizeof(SMagnetometerCalibrationParameters3);
  static constexpr uint16_t skuBlockOffsetPressureCompensationParameters = skuBlockOffsetCanConfigurationParameters + sizeof(SCanConfigurationParameters);


  template<typename tBlockDataType, unsigned uOffset> struct TNvmBlock
  {
    using CDataType = tBlockDataType;
    static constexpr uint16_t skuOffset_ { uOffset };
  };

  using CNvmBlockNmvFormatVersion = TNvmBlock<SNmvFormatVersion, skuBlockOffsetNmvFormatVersion>;
  using CNvmBlockEulerAnglesDeviceToVehicle = TNvmBlock<SEulerAnglesDeviceToVehicle, skuBlockOffsetEulerAnglesDeviceToVehicle>;
  using CNvmBlockMagnetometerCalibrationParameters1 = TNvmBlock<SMagnetometerCalibrationParameters1, skuBlockOffsetMagnetometerCalibrationParameters1>;
  using CNvmBlockMagnetometerCalibrationParameters2 = TNvmBlock<SMagnetometerCalibrationParameters2, skuBlockOffsetMagnetometerCalibrationParameters2>;
  using CNvmBlockMagnetometerCalibrationParameters3 = TNvmBlock<SMagnetometerCalibrationParameters3, skuBlockOffsetMagnetometerCalibrationParameters3>;
  using CNvmBlockCanConfigurationParameters = TNvmBlock<SCanConfigurationParameters, skuBlockOffsetCanConfigurationParameters>;
  using CNvmBlockPressureCompensationParameters = TNvmBlock<SPressureCompensationParameters, skuBlockOffsetPressureCompensationParameters>;


  template<typename tNvmBlock> static bool ReadBlock(typename tNvmBlock::CDataType& orBlockData)
  {
    bool bStatus = false;
    uint8_t auPacket[2];

    uint16_t uMemoryAddress = tNvmBlock::skuOffset_;

    auPacket[0] = reinterpret_cast<uint8_t*>(&uMemoryAddress)[1];
    auPacket[1] = reinterpret_cast<uint8_t*>(&uMemoryAddress)[0];

    HAL_StatusTypeDef eSts = HAL_FMPI2C_Master_Transmit(&hfmpi2c1,
                                                        static_cast<uint16_t>(0x50) << 1,
                                                        auPacket, 2, 1);

    if (HAL_OK == eSts)
    {
      eSts = HAL_FMPI2C_Master_Receive(&hfmpi2c1,
                                       static_cast<uint16_t>(0x50) << 1,
                                       reinterpret_cast<uint8_t*>(&orBlockData),
                                       sizeof(orBlockData), sizeof(orBlockData));

      if (eSts == HAL_OK)
      {
        bStatus = true;
      }
    }

    return bStatus;
  }

  template<typename tNvmBlock> static bool WriteBlock(const typename tNvmBlock::CDataType& korBlockData)
  {
    bool bStatus = true;
    uint8_t auPacket[3];

    uint16_t uMemoryAddress = tNvmBlock::skuOffset_;

    // Enable write operation
    HAL_GPIO_WritePin(NVM_WRITE_CONTROL_GPIO_Port, NVM_WRITE_CONTROL_Pin, GPIO_PIN_RESET);

    for(uint16_t uByteCount = 0U; uByteCount < sizeof(korBlockData); ++uByteCount)
    {
      auPacket[0] = reinterpret_cast<uint8_t*>(&uMemoryAddress)[1];
      auPacket[1] = reinterpret_cast<uint8_t*>(&uMemoryAddress)[0];
      auPacket[2] = reinterpret_cast<const uint8_t*>(&korBlockData)[uByteCount];

      HAL_StatusTypeDef eWriteStatus = HAL_FMPI2C_Master_Transmit(&hfmpi2c1,
        static_cast<uint16_t>(0x50) << 1,
        auPacket, 3U, 1U);

      HAL_Delay(3);

      if(HAL_OK != eWriteStatus)
      {
        bStatus = false;
        break;
      }

      ++uMemoryAddress;
    }

    // Enable write protection
    HAL_GPIO_WritePin(NVM_WRITE_CONTROL_GPIO_Port, NVM_WRITE_CONTROL_Pin, GPIO_PIN_SET);

    return bStatus;
  }

  static bool ReadPage(uint16_t uPageNumber, uint8_t* upPage)
  {
    bool bStatus = true;

    if ((nullptr == upPage) || (uPageNumber >= skuMemorySize_ / skuPageSize_))
    {
      bStatus = false;
    }

    if (true == bStatus)
    {
      uint8_t auPacket[2];
      uint16_t uMemoryAddress = uPageNumber * skuPageSize_;

      auPacket[0] = reinterpret_cast<uint8_t*>(&uMemoryAddress)[1];
      auPacket[1] = reinterpret_cast<uint8_t*>(&uMemoryAddress)[0];

      HAL_StatusTypeDef eSts = HAL_FMPI2C_Master_Transmit(&hfmpi2c1,
                                                          static_cast<uint16_t>(0x50) << 1,
                                                          auPacket, 2, 1);

      if (HAL_OK == eSts)
      {
        eSts = HAL_FMPI2C_Master_Receive(&hfmpi2c1,
                                         static_cast<uint16_t>(0x50) << 1,
                                         upPage, skuPageSize_, skuPageSize_);
      }

      if(HAL_OK != eSts)
      {
        bStatus = false;
      }
    }

    return bStatus;
  }

  static bool WritePage(uint16_t uPageNumber, uint8_t* upPage)
  {
    bool bStatus = true;

    if ((nullptr == upPage) || (uPageNumber >= skuMemorySize_ / skuPageSize_))
    {
      bStatus = false;
    }

    if (true == bStatus)
    {
      // Enable write operation
      HAL_GPIO_WritePin(NVM_WRITE_CONTROL_GPIO_Port, NVM_WRITE_CONTROL_Pin, GPIO_PIN_RESET);

      uint16_t uMemoryAddress = uPageNumber * skuPageSize_;
      uint8_t auBuffer[2U + skuPageSize_];

      auBuffer[0] = reinterpret_cast<uint8_t*>(&uMemoryAddress)[1];
      auBuffer[1] = reinterpret_cast<uint8_t*>(&uMemoryAddress)[0];
      memcpy(&(auBuffer[2]), upPage, skuPageSize_);

      HAL_StatusTypeDef eWriteStatus = HAL_FMPI2C_Master_Transmit(&hfmpi2c1,
                                                                  static_cast<uint16_t>(0x50) << 1,
                                                                  auBuffer, sizeof(auBuffer), 3 * sizeof(auBuffer));

      if(HAL_OK != eWriteStatus)
      {
        bStatus = false;
      }

      // Enable write protection
      HAL_GPIO_WritePin(NVM_WRITE_CONTROL_GPIO_Port, NVM_WRITE_CONTROL_Pin, GPIO_PIN_SET);
    }

    return bStatus;
  }

protected:
private:
  CNvmDriver() = default;
  ~CNvmDriver() = default;
};

#endif /* C_NVM_DRIVER_H */
