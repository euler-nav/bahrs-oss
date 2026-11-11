/// @file CCanHandler.h
/// @brief Declaration of the CCanHandler class (CAN general handler software component).
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_CAN_HANDLER_H
#define C_CAN_HANDLER_H

#include "canaerospace/canaerospace.h"
#include "canaerospace/param_id/nod_default.h"
#include "canaerospace/services/std_identification.h"
#include "can_driver.h"
#include "GetMicroseconds.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "General/CSoftwareComponentBase.h"

typedef StaticSemaphore_t osStaticMutexDef_t;

/// @brief CAN General Handler software component.
class CCanHandler : public CSoftwareComponent<CCanHandler, 1U>
{
  friend class CSoftwareComponent<CCanHandler, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CCanHandler)
  DECLARE_MANDATORY_APIS(CCanHandler)

public:
  static constexpr int32_t skiMallocSize { 1024 }; ///< Static Malloc size in bytes
  static constexpr uint32_t skuMutexTout { 1 };    ///< CAN Aerospace mutex timeout

  /// @brief Platform-specific data.
  struct SCanasPlatform
  {
    uint8_t auMallocPool[skiMallocSize] __attribute__((aligned(__BIGGEST_ALIGNMENT__)));
    int32_t iMallocOffset;
    osMutexId_t pMutex;
    osStaticMutexDef_t oMutexcb;
  };

  CanasInstance oCanasInstance;                     ///< CAN Aerospace instance object.
  SCanasPlatform oCanasPlatform;                    ///< CAN Aerospace Platform specific.

  /// @brief CAN Listener
  static void ReceiveCanFrames();

private:
  CCanHandler();
  ~CCanHandler() = default;

  bool bIsInitialized_ { false };
  osMutexAttr_t oMutexAttr_;

  /// @brief Send a message to the bus.
  /// @param [in] opInstance     Instance pointer
  /// @param [in] iIface         Interface index
  /// @param [in] kopFrame       Pointer to the message to be sent
  /// @return                    Number of messages sent (0 or 1), negative on failure
  static int drvSend(CanasInstance* opInstance, int iIface, const CanasCanFrame* kopFrame)
  {
    assert(opInstance);
    assert(iIface >= 0 && iIface < CAN_IFACE_COUNT);
    assert(kopFrame);
    return canSend(iIface, kopFrame);
  }

  /// @brief Configure acceptance filters.
  /// @param [in] opInstance     Instance pointer
  /// @param [in] iIface         Interface index
  /// @param [in] kopFilters     Pointer to the filter configuration
  /// @param [in] iNumFilt       Number of configured filters
  /// @return                    0 if ok, negative on failure
  static int drvFilter(CanasInstance* opInstance, int iIface, const CanasCanFilterConfig* kopFilters, int iNumFilt);

  /// @brief Allocates a chunk of memory.
  /// If the application does not require de-initialization features like unsubscription of unadvertisement, then dynamic
  /// memory is not needed at all. If this is the case, you can use a static pool allocator.
  /// This feature is useful for embedded systems where dynamic memory is not always available.
  /// @note Allocated memory must be aligned properly.
  /// @param [in] opInstance     Instance pointer
  /// @param [in] iSize          Size of the memory block required
  /// @return                    Pointer or NULL if no memory available
  static void* staticMalloc(CanasInstance* opInstance, int iSize);

  /// @brief Returns current timestamp in microseconds.
  /// Any base value (uptime or UNIX time) will do.
  /// @param [in] opInstance     Instance pointer
  /// @return                    Current timestamp in microseconds
  static uint64_t getTimestamp(CanasInstance* opInstance);
};

#endif // C_CAN_HANDLER_H
