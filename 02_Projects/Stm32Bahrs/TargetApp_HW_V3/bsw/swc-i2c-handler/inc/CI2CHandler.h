/// @file CI2CHandler.h
/// @brief Declaration of the I2C handler software component class.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_I2C_HANDLER_H
#define C_I2C_HANDLER_H

#include <array>
#include <cstdint>

#include "General/CSoftwareComponentBase.h"
#include "CMutex.h"
#include "i2c.h"

/// @brief Software component class for handling I2C communication with peripherals.
class CI2CHandler : public CSoftwareComponent<CI2CHandler, 1U>
{
  friend class CSoftwareComponent<CI2CHandler, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CI2CHandler)
  DECLARE_MANDATORY_APIS(CI2CHandler)

public:
  /// @brief Enumeration of supported I2C buses.
  enum class EBus : uint8_t
  {
    eI2c1 = 0U,
    eI2c2 = 1U,
    eI2c3 = 2U
  };

  /// @brief Read from an I2C device memory register.
  /// @return True on success, false otherwise.
  bool MemRead(EBus eBus, uint16_t uDevAddr, uint16_t uMemAddr, uint16_t uMemAddrSize,
               uint8_t* upData, uint16_t uSize, uint32_t uTimeout);

  /// @brief Write to an I2C device memory register.
  /// @return True on success, false otherwise.
  bool MemWrite(EBus eBus, uint16_t uDevAddr, uint16_t uMemAddr, uint16_t uMemAddrSize,
                const uint8_t* upData, uint16_t uSize, uint32_t uTimeout);

private:
  CI2CHandler() = default;
  ~CI2CHandler() = default;

  static constexpr uint8_t skuBusCount_{ 3U };
  static constexpr uint8_t skuFailureThreshold_{ 10U };
  static constexpr uint32_t skuMutexAcquisitionTimeout_{ 2U };
  static constexpr uint32_t skuWaitOnFlagTimeout_{ 25U };
  static constexpr osPriority_t skeDemotedPriority_{ osPriorityNormal }; // Must be below Task5ms (AboveNormal1) and TaskRs232Sender (AboveNormal).

  struct SBusState
  {
    CMutex oMutex; ///< Mutex for synchronizing access to the bus.
    uint8_t uConsecutiveFailures_{ 0U }; ///< Number of consecutive BUSY/ERROR results.
    bool bDemoted_{ false }; ///< True once the calling task has been demoted.
  };

  bool mutexAcquire(EBus eBus);

  void mutexRelease(EBus eBus);

  static bool i2cWaitOnFlag(I2C_HandleTypeDef* opHandle,
                            uint32_t uFlag,
                            FlagStatus eStatus);

  static I2C_HandleTypeDef* getI2CHandle(EBus eBus);

  void updateFailureState(EBus eBus, HAL_StatusTypeDef eStatus);

  static void demoteTasks();

  SBusState& getBusState(EBus eBus);

  const SBusState& getBusState(EBus eBus) const;

  bool bIsInitialized_{ false };
  std::array<SBusState, skuBusCount_> aoBusStates_{};
};

#endif // C_I2C_HANDLER_H
