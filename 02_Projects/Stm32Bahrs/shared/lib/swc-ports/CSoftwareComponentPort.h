/// @file CSoftwareComponentPort.h
/// @brief Declaration of software component port template.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_SOFTWARE_COMPONENT_PORT_H
#define C_SOFTWARE_COMPONENT_PORT_H

#ifndef _MSC_VER
#include "cmsis_os.h"
#else
#include <stdint.h>
#endif // _MSC_VER

#include "GetMicroseconds.h"

#ifdef SEND_DEBUG_OUTPUT
#include "UintToBool.h"

#ifndef _MSC_VER
#include "CRs232OutputHandler.h"
using CDebugOutputHandler = CRs232OutputHandler;
#else
#include "CFileDebugOutputHandler.h"
using CDebugOutputHandler = CFileDebugOutputHandler;
#endif // _MSC_VER

#endif // SEND_DEBUG_OUTPUT

#include "General/HelperSoftwareComponentMacros.h"
#include "DataSerializer.h"
#include <array>
#include <functional>
#include <optional>
#include <type_traits>
#include "AmsAssert.h"
#include "TemplateUtils.h"
#include <tuple>

template<typename... TPortTypes> struct TArePortIdsAscending;

template<typename PortType>
struct TArePortIdsAscending<PortType>
{
  static constexpr bool bValue_{true};
};

template<typename FirstPortType, typename... RestPortTypes>
struct TArePortIdsAscending<FirstPortType, RestPortTypes...>
{
  static constexpr bool bValue_{
    (FirstPortType::skuPortId_ < TFirstTypeInPack<RestPortTypes...>::Type::skuPortId_) &&
    TArePortIdsAscending<RestPortTypes...>::bValue_ };
};

class CSoftwareComponentPortBase
{
public:
  /// @brief Constructor. Creates a mutex for the port.
  /// The function will assert if the mutex creation fails.
  explicit CSoftwareComponentPortBase();

  ~CSoftwareComponentPortBase();

  FORBID_CLASS_COPY_AND_MOVE(CSoftwareComponentPortBase)

  /// Initialize the port. Deprecated.
  /// Left for compatibility with th RTE generator.
  /// @return True -- success, false -- failure.
  bool Init()
  {
    return true;
  }

  /// Lock the port for exclusive access.
  /// @return True -- success, false -- failure.
  bool Lock();

  /// Unlock the port.
  void Unlock();

  /// Get port unique identifier.
  virtual uint8_t GetId() const = 0;

#ifdef SEND_DEBUG_OUTPUT
  /// @brief Enable/disable port recording in debug output variant.
  /// @param bFlag True -- enable recording, false -- disable.
  inline void SetRecordForDebug(bool bFlag)
  {
    bRecordForDebug_ = bFlag;
  }
#endif // SEND_DEBUG_OUTPUT

protected:
  static constexpr uint32_t skuPortLockTimeout{ 1U }; ///< Timeout for port lock operation (in FreeRTOS ticks)

#ifndef _MSC_VER
  // The typedef copied from a CMSIS_OS header.
  using osStaticMutexDef_t = StaticSemaphore_t;

  osMutexId_t pMutexHandle_; ///< Mutex handle.
  osStaticMutexDef_t sMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t sMutexAttributes_ { NULL, 0, &sMutexControlBlock_, sizeof(sMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.
#endif // _MSC_VER

#ifdef SEND_DEBUG_OUTPUT
  bool bRecordForDebug_ { false };
#endif // SEND_DEBUG_OUTPUT
};

class CPortReader
{
public:
  /// Read data from multiple ports.
  /// @tparam TRunnableId Runnable ID type (enum or integer)
  /// @tparam TPortTypes Variadic list of port types.
  /// @param eRunnableId Runnable ID. Use 0 to skip logging, or if not applicable.
  /// @param orPorts Variadic list of port references.
  /// @return An optional tuple containing the data read from the ports. If any port fails to lock, returns empty optional.
  template<typename TRunnableId, typename... TPortTypes>
  static auto ReadPorts(TRunnableId eRunnableId, TPortTypes&... orPorts) -> std::optional<std::tuple<typename TPortTypes::DataType...>>;
};

/// The class implements a software component port. The port is a mutex-protected variable used by
/// software components to pass data from one to another.
template<class tPortDataType, uint8_t uPortId, uint32_t uInvalidationTimeoutMs = 0U> class CSoftwareComponentPort : public CSoftwareComponentPortBase
{
public:
  CSoftwareComponentPort() = default;

  ~CSoftwareComponentPort() = default;

  FORBID_CLASS_COPY_AND_MOVE(CSoftwareComponentPort)

  /// Write data to the port.
  /// @param korData A reference to the data to be written to the port.
  /// @return True -- success, false -- failure.  
  bool Write(const tPortDataType& korData);

  /// Read from the port.
  /// @param orData A reference to the output data structure.
  /// @return True -- success, false -- failure.
  bool Read(tPortDataType& orData);

  uint8_t GetId() const final
  {
    return skuPortId_;
  }

  using DataType = tPortDataType;

private:
  void invalidateDataIfTimeoutExceeded()
  {
    if constexpr (0U < skuInvalidationTimeoutUs_)
    {
      const uint64_t kuCurrentTimestampUs{GetMicroseconds()};

      if (uLastWriteTimestampUs_ <= kuCurrentTimestampUs)
      {
        if (skuInvalidationTimeoutUs_ < (kuCurrentTimestampUs - uLastWriteTimestampUs_))
        {
          // Invalidation is treated as a write of default data: it restarts the timeout and is logged
          // as a regular port write event, so the replay tool reproduces it without modifications.
          oPortData_ = tPortDataType{};
          uLastWriteTimestampUs_ = kuCurrentTimestampUs;

#ifdef SEND_DEBUG_OUTPUT
          sendWriteDebugEvent(oPortData_, kuCurrentTimestampUs);
#endif // SEND_DEBUG_OUTPUT
        }
      }
    }
  }

#ifdef SEND_DEBUG_OUTPUT
  /// @brief Send a port write debug event if the port is recorded for debug.
  /// @param orData Port data to serialize (non-const because ToByteVector() is non-const).
  /// @param uTimestampUs Timestamp [us] of the event.
  void sendWriteDebugEvent(tPortDataType& orData, uint64_t uTimestampUs)
  {
    if (bRecordForDebug_)
    {
      CSerialProtocol::SDebugData oDebugData;
      oDebugData.uMsgType_ = CSerialProtocol::EMessageIds::eDebugEventWriteToPort;
      oDebugData.uId_ = uPortId;
      oDebugData.uTimestampUs_ = uTimestampUs;
      oDebugData.oByteVector_ = orData.ToByteVector();

      if (oDebugData.oByteVector_.size() > 0)
      {
        CDebugOutputHandler::GetInstance().SendDebugData(oDebugData);
      }
    }
  }
#endif // SEND_DEBUG_OUTPUT

  tPortDataType readUnlocked()
  {
    invalidateDataIfTimeoutExceeded();
    return oPortData_;
  }

  static constexpr uint8_t skuPortId_{uPortId}; ///< Port unique identifier.
  static constexpr uint64_t skuInvalidationTimeoutUs_{static_cast<uint64_t>(uInvalidationTimeoutMs) * 1000U}; ///< Port data invalidation timeout [us]. 0 means disabled.
  tPortDataType oPortData_{}; ///< Port data storage.
  uint64_t uLastWriteTimestampUs_{0U}; ///< Timestamp [us] of the latest successful write or invalidation.

  friend class CPortReader;
  template<typename... TPortTypes> friend struct TArePortIdsAscending;
};

template<typename tPortDataType, uint8_t uPortId, uint32_t uInvalidationTimeoutMs>
bool CSoftwareComponentPort<tPortDataType, uPortId, uInvalidationTimeoutMs>::Write(const tPortDataType& korData)
{
  bool bRetVal = false;

  if (Lock())
  {
    oPortData_ = korData;
    uLastWriteTimestampUs_ = GetMicroseconds();

#ifdef SEND_DEBUG_OUTPUT
    auto oPortDataTmp_ = oPortData_;
    const uint64_t kuWriteTimestampUsTmp{uLastWriteTimestampUs_};
#endif // SEND_DEBUG_OUTPUT

    Unlock();
    bRetVal = true;

#ifdef SEND_DEBUG_OUTPUT
    sendWriteDebugEvent(oPortDataTmp_, kuWriteTimestampUsTmp);
#endif // SEND_DEBUG_OUTPUT
  }

  return bRetVal;
}

template<typename tPortDataType, uint8_t uPortId, uint32_t uInvalidationTimeoutMs>
bool CSoftwareComponentPort<tPortDataType, uPortId, uInvalidationTimeoutMs>::Read(tPortDataType& orData)
{
  bool bRetVal = false;

  if (Lock())
  {
    orData = readUnlocked();
    Unlock();
    bRetVal = true;
  }

  return bRetVal;
}

template<typename TRunnableId, typename... TPortTypes>
auto CPortReader::ReadPorts(TRunnableId eRunnableId, TPortTypes&... orPorts) -> std::optional<std::tuple<typename TPortTypes::DataType...>>
{
  static_assert(std::is_enum<TRunnableId>::value || std::is_integral<TRunnableId>::value,
                "Runnable ID must be an enum or an integer type.");
  static_assert(TArePortIdsAscending<TPortTypes...>::bValue_, "Ports must be read in ascending order of port IDs!");

  using COutputTupleType = std::optional<std::tuple<typename TPortTypes::DataType...>>;
  COutputTupleType oReadOutputs{};

  // Lock status: 2 -- locked, 1 -- not locked, 0 -- not attempted to lock.
  enum class ELockStatus : uint8_t
  {
    eNotAttempted = 0U,
    eNotLocked = 1U,
    eLocked = 2U
  };

  std::array<ELockStatus, sizeof...(TPortTypes)> oLockStatus;
  oLockStatus.fill(ELockStatus::eNotAttempted);

  bool bAllLocked{ true };
  uint32_t uIndex{ 0U };

  auto lockAndRecordStatus = [&oLockStatus, &uIndex, &bAllLocked](auto& orPort)
    {
      if (orPort.Lock())
      {
        oLockStatus[uIndex] = ELockStatus::eLocked;
      }
      else
      {
        oLockStatus[uIndex] = ELockStatus::eNotLocked;
        bAllLocked = false;
      }

      ++uIndex;

      return bAllLocked;
    };

  // We lock in the first to last order and stop on the first failure
  (true && ... && lockAndRecordStatus(orPorts));

#ifdef SEND_DEBUG_OUTPUT
  const uint8_t uRunnableId = static_cast<uint8_t>(eRunnableId);
  if (0U != uRunnableId)
  {
    CSerialProtocol::SDebugData oDebugData;
    oDebugData.uMsgType_ = CSerialProtocol::EMessageIds::eDebugEventRunnableCall;
    oDebugData.uId_ = uRunnableId;
    oDebugData.uTimestampUs_ = GetMicroseconds();
    oDebugData.bAllPortsLocked_ = BoolToUint(bAllLocked);

    CDebugOutputHandler::GetInstance().SendDebugData(oDebugData);
  }
#else
  (void)eRunnableId;
#endif // SEND_DEBUG_OUTPUT

  if (bAllLocked)
  {
    oReadOutputs = std::make_tuple(orPorts.readUnlocked()...);
  }

  auto unlockIfLocked = [&oLockStatus, &uIndex](auto& orPort)
    {
      if (ELockStatus::eLocked == oLockStatus[uIndex])
      {
        orPort.Unlock();
      }

      ++uIndex;
    };

  uIndex = 0U;
  (void(), ..., unlockIfLocked(orPorts));

  return oReadOutputs;
}

#endif // C_SOFTWARE_COMPONENT_PORT_H
