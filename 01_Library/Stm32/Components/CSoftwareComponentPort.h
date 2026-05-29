/// @file CSoftwareComponentPort.h
/// @brief Declaration of software component port template.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_SOFTWARE_COMPONENT_PORT_H
#define C_SOFTWARE_COMPONENT_PORT_H


#include "cmsis_os.h"
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

protected:
  static constexpr uint32_t skuPortLockTimeout{ 1U }; ///< Timeout for port lock operation (in FreeRTOS ticks)

  // The typedef copied from a CMSIS_OS header.
  using osStaticMutexDef_t = StaticSemaphore_t;

  osMutexId_t pMutexHandle_; ///< Mutex handle.
  osStaticMutexDef_t sMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t sMutexAttributes_ { NULL, 0, &sMutexControlBlock_, sizeof(sMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.
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
template<class tPortDataType, uint8_t uPortId> class CSoftwareComponentPort : public CSoftwareComponentPortBase
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
  tPortDataType readUnlocked()
  {
    return oPortData_;
  }

  static constexpr uint8_t skuPortId_{uPortId}; ///< Port unique identifier.
  tPortDataType oPortData_{}; ///< Port data storage.

  friend class CPortReader;
  template<typename... TPortTypes> friend struct TArePortIdsAscending;
};

template<typename tPortDataType, uint8_t uPortId>
bool CSoftwareComponentPort<tPortDataType, uPortId>::Write(const tPortDataType& korData)
{
  bool bRetVal = false;

  if (Lock())
  {
    oPortData_ = korData;
    Unlock();
    bRetVal = true;
  }

  return bRetVal;
}

template<typename tPortDataType, uint8_t uPortId>
bool CSoftwareComponentPort<tPortDataType, uPortId>::Read(tPortDataType& orData)
{
  bool bRetVal = false;

  if (Lock())
  {
    orData = oPortData_;
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

  (void)eRunnableId;

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
