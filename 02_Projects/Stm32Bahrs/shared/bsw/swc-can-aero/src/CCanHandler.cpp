/// @file CCanHandler.cpp
/// @brief Implementation of the CAN Global Handler software component.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CCanHandler.h"
#include "CRte.h"

#include "CNvmDriver.h"
#include "crc32.h"

CCanHandler& CCanHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CCanHandler soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

CCanHandler::CCanHandler()
{
  // Do nothing
}

void CCanHandler::Init()
{
  bool bStatus = false;

  if (false == bIsInitialized_)
  {
    CanasConfig oCfg;
    CanasSrvIdsPayload oIds;
    CNvmDriver::SCanConfigurationParameters oCanConfigParameters;

    uint32_t uNodeId      = 45;
    uint32_t uRedundantCh = 0;
    uint32_t uServiceCh   = 8;

    uint8_t uHardwareRev = 0x01U;
    uint8_t uSoftwareRev = 0x01U;

    if(true == CNvmDriver::ReadBlock<CNvmDriver::CNvmBlockCanConfigurationParameters>(oCanConfigParameters))
    {
      Crc32_t uComputedCrc = crc32(reinterpret_cast<void*>(&oCanConfigParameters), sizeof(oCanConfigParameters) - sizeof(Crc32_t));

      if (uComputedCrc == oCanConfigParameters.uCrc_)
      {
        uNodeId      = oCanConfigParameters.uNodeID_;
        uRedundantCh = oCanConfigParameters.uRedundantChannelID_;
        uServiceCh   = oCanConfigParameters.uServiceChannelID_;
      }
    }

    // Try CAN low level initialization. CAN TX timeout is set to a large value because it is not really needed.
    // This is because we plan to send CAN frames cyclically, at high rate and in non-blocking way. The timer-based
    // timeout detection strategy implemented in the 3-rd party CANaerospace stack seems impractical for this use-case.
    // TODO: consider removing CAN timer and all associated resources.
    bStatus = (0 == canInit(500000, 1000000)) ? true : false;

    if (true == bStatus)
    {
      oCfg = canasMakeConfig();
      oCfg.filters_per_iface = CAN_FILTERS_PER_IFACE;
      oCfg.iface_count       = CAN_IFACE_COUNT;
      oCfg.fn_send           = drvSend;
      oCfg.fn_filter         = drvFilter;
      oCfg.fn_malloc         = staticMalloc;        // No dynamic memory needed. Thus, free() is not necessary too.
      oCfg.fn_timestamp      = getTimestamp;
      // Each function pointer is initialised to NULL, so no need to assign them by hand.
      // These settings are application-defined:
      oCfg.node_id           = static_cast<uint8_t>(uNodeId);
      oCfg.redund_channel_id = static_cast<uint8_t>(uRedundantCh);
      oCfg.service_channel   = static_cast<uint8_t>(uServiceCh);

      memset(&oCanasPlatform, 0, sizeof(oCanasPlatform));

      oMutexAttr_.name = "CanasMutex";
      oMutexAttr_.cb_mem = &oCanasPlatform.oMutexcb;
      oMutexAttr_.cb_size = sizeof(oCanasPlatform.oMutexcb);
      oCanasPlatform.pMutex = osMutexNew(&oMutexAttr_);
      bStatus = (oCanasPlatform.pMutex != NULL) ? true : false;
    }

    // Try CAN Aerospace protocol initialization
    if (true == bStatus)
    {
      bStatus = (0 == canasInit(&oCanasInstance, &oCfg, &oCanasPlatform)) ? true : false;
      if (true == bStatus)
      {
        // IDS service (it is mandatory for every node):
        oIds.hardware_revision = uHardwareRev;
        oIds.software_revision = uSoftwareRev;
        oIds.id_distribution   = CANAS_SRV_IDS_ID_DISTRIBUTION_STD;  // These two are standard
        oIds.header_type       = CANAS_SRV_IDS_HEADER_TYPE_STD;
        bStatus = (0 == canasSrvIdsInit(&oCanasInstance, &oIds, 0)) ? true : false;
      }
    }

    // Can output is initialized
    if (true == bStatus)
    {
      bIsInitialized_ = true;
    }
  }
}

bool CCanHandler::IsInitialized()
{
  return bIsInitialized_;
}

void CCanHandler::ReceiveCanFrames()
{
  static constexpr int32_t skiUpdateIntervalMicroseconds { 10 * 1000 };
  CanasCanFrame oFrame;
  int iIface = -1;
  CCanHandler& orInstance = GetInstance();

  // Note that CAN driver is fully reentrant, and it is possible to perform
  // blocking canReceive() and canSend() (or whatever) at the same time.
  // canReceive() will read data from a FreeRTOS queue, so the calling
  // task can be blocked for up to skiUpdateIntervalMicroseconds.
  int iRet = canReceive(&iIface, &oFrame, skiUpdateIntervalMicroseconds);
  if (0 <= iRet)
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, skuMutexTout))
    {
      if (iRet == 0)
      {
        // No data received
        iRet = canasUpdate(&orInstance.oCanasInstance, -1, NULL);
      }
      else
      {
        // Pass a new frame
        assert(iIface >= 0 && iIface < CAN_IFACE_COUNT);
        iRet = canasUpdate(&orInstance.oCanasInstance, iIface, &oFrame);
      }
      osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

int CCanHandler::drvFilter(CanasInstance* opInstance, int iIface, const CanasCanFilterConfig* kopFilters, int iNumFilt)
{
  assert(opInstance);
  assert(iIface >= 0 && iIface < CAN_IFACE_COUNT);
  assert(kopFilters);
  assert(iNumFilt > 0);
  return canFilterSetup(iIface, kopFilters, iNumFilt);
}

void* CCanHandler::staticMalloc(CanasInstance* opInstance, int iSize)
{
  assert(opInstance);
  assert(iSize > 0);
  SCanasPlatform* opPlatform = reinterpret_cast<SCanasPlatform*>(opInstance->pthis);
  assert(opPlatform);

  // Proper alignment is absolutely vital:
  assert((sizeof(opPlatform->auMallocPool) % __BIGGEST_ALIGNMENT__) == 0);

  if ((iSize % __BIGGEST_ALIGNMENT__) != 0)
  {
    iSize += (__BIGGEST_ALIGNMENT__ - iSize % __BIGGEST_ALIGNMENT__);
  }

  if (iSize >= (skiMallocSize - opPlatform->iMallocOffset))
  {
    return NULL;
  }

  void* pPointer = opPlatform->auMallocPool + opPlatform->iMallocOffset;
  opPlatform->iMallocOffset += iSize;

  return pPointer;
}

uint64_t CCanHandler::getTimestamp(CanasInstance* opInstance)
{
  (void)opInstance;
  return GetMicroseconds();
}
