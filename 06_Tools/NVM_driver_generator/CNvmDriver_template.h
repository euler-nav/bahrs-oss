/** ---- IMPORTANT ----
 *      
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * Date: {DATE_GENERATED}
*/
#ifndef {INCLUSION_GUARD}
#define {INCLUSION_GUARD}

#include <stdint.h>
#include <memory.h>

#include "main.h"
#include "stm32f4xx_hal_fmpi2c.h"
#include "General/CSoftwareComponentBase.h"

extern FMPI2C_HandleTypeDef hfmpi2c1;

class {CLASS_NAME} : public CSoftwareComponent<{CLASS_NAME}, 0U>
{
  friend class CSoftwareComponent<{CLASS_NAME}, 0U>;
  FORBID_CLASS_COPY_AND_MOVE({CLASS_NAME})
  DECLARE_MANDATORY_APIS({CLASS_NAME})

public:

  static constexpr uint16_t skuMemorySize_ { {MEMORY_SIZE} };
  static constexpr uint8_t skuPageSize_ { 32U };
  static constexpr uint16_t skuVersionMajor_ { {VERSION_MAJOR} };
  static constexpr uint16_t skuVersionMinor_ { {VERSION_MINOR} };

#pragma pack(push, {STRUCT_ALIGNMENT})

{DECLARATION_OF_BLOCK_TYPES}
{DECLARATION_OF_FULL_NVM_STRUCT}
  static_assert(sizeof({NVM_FULL_STRUCT_NAME}) <= skuMemorySize_);

#pragma pack(pop)

{DECLARATION_OF_BLOCK_OFFSETS}

  template<typename tBlockDataType, unsigned uOffset> struct TNvmBlock
  {
    using CDataType = tBlockDataType;
    static constexpr uint16_t skuOffset_ { uOffset };
  };

{DECLARATION_OF_BLOCK_OBJECTS}

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
  {CLASS_NAME}() = default;
  ~{CLASS_NAME}() = default;
};

#endif /* {INCLUSION_GUARD} */
