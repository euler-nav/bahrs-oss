/// @file CDroneCanHandler.cpp
/// @brief Implementation of the CDroneCanHandler class.
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

// The DroneCAN stack is a build variant: this translation unit is empty
// unless USE_DRONECAN is defined.
#ifdef USE_DRONECAN

#include "CDroneCanHandler.h"
#include "CDroneCanOutputHandler.h"
#include "DroneCanDsdlMessages.h"
#include "bxcan.h"
#include "CRte.h"
#include "CNvmDriver.h"
#include "CSoftwareVersion.h"
#include "crc32.h"
#include "stm32f4xx_hal.h"  // HAL_GetUIDwX() for the GetNodeInfo unique ID
#include <string.h>

namespace
{
  /// DroneCAN node name reported in the GetNodeInfo response.
  constexpr char skacNodeName[] = "ams.bahrs";

  /// Hardware version reported in the GetNodeInfo response (informational).
#ifdef BAHRS_HW_V3
  constexpr uint8_t skuHardwareVersionMajor { 3U };
#else
  constexpr uint8_t skuHardwareVersionMajor { 2U };
#endif
  constexpr uint8_t skuHardwareVersionMinor { 0U };
}

CDroneCanHandler& CDroneCanHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CDroneCanHandler soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

CDroneCanHandler::CDroneCanHandler()
{
  // Do nothing - member variables are initialised in place
}

void CDroneCanHandler::Init()
{
  if (false == bIsInitialized_)
  {
    bool bStatus = false;

    uint8_t uNodeId = 45U; // Default DroneCAN node ID

    // Reuse the existing CAN configuration NVM block for the node ID.
    CNvmDriver::SCanConfigurationParameters oCanConfigParameters;
    if (CNvmDriver::ReadBlock<CNvmDriver::CNvmBlockCanConfigurationParameters>(oCanConfigParameters))
    {
      const Crc32_t uComputedCrc = crc32(
        reinterpret_cast<void*>(&oCanConfigParameters),
        sizeof(oCanConfigParameters) - sizeof(Crc32_t));

      if (uComputedCrc == oCanConfigParameters.uCrc_)
      {
        uNodeId = static_cast<uint8_t>(oCanConfigParameters.uNodeID_);
      }
    }

    // Initialise the bxCAN hardware (500 kbit/s, accept-all filter, polled I/O).
    bStatus = BxCan_Init();

    if (bStatus)
    {
      // Initialise the libcanard instance with the static memory pool.
      canardInit(&oCanardInstance_,
                 auMemoryPool_,
                 static_cast<size_t>(skuMemoryPoolSize),
                 onTransferReception_,
                 shouldAcceptTransfer_,
                 nullptr);

      canardSetLocalNodeID(&oCanardInstance_, uNodeId);

      // Create FreeRTOS mutex for thread-safe access to the libcanard instance.
      oMutexAttr_.name    = "DroneCanMutex";
      oMutexAttr_.cb_mem  = &oMutexCb_;
      oMutexAttr_.cb_size = sizeof(oMutexCb_);
      pMutex_ = osMutexNew(&oMutexAttr_);
      bStatus = (pMutex_ != NULL);
    }

    if (bStatus)
    {
      bIsInitialized_ = true;
    }
  }
}

bool CDroneCanHandler::IsInitialized()
{
  return bIsInitialized_;
}

int16_t CDroneCanHandler::Broadcast(uint64_t uDataTypeSignature,
                                     uint16_t uDataTypeId,
                                     uint8_t* upTransferId,
                                     uint8_t  uPriority,
                                     const void* kpPayload,
                                     uint16_t uPayloadLen)
{
  int16_t iResult = -1;

  if (osOK == osMutexAcquire(pMutex_, skuMutexTout))
  {
    iResult = canardBroadcast(&oCanardInstance_,
                               uDataTypeSignature,
                               uDataTypeId,
                               upTransferId,
                               uPriority,
                               kpPayload,
                               static_cast<uint16_t>(uPayloadLen));

    osMutexRelease(pMutex_);
  }

  return iResult;
}

void CDroneCanHandler::TransmitFrames()
{
  if (osOK == osMutexAcquire(pMutex_, skuMutexTout))
  {
    for (;;)
    {
      const CanardCANFrame* kpFrame = canardPeekTxQueue(&oCanardInstance_);
      if (kpFrame == NULL)
      {
        break;
      }

      // DroneCAN always uses 29-bit extended IDs; take the low 29 bits.
      const uint32_t uExtId = kpFrame->id & CANARD_CAN_EXT_ID_MASK;

      if (BxCan_Send(uExtId, kpFrame->data, kpFrame->data_len))
      {
        canardPopTxQueue(&oCanardInstance_);
      }
      else
      {
        // All TX mailboxes are busy - keep the frame in the libcanard queue.
        break;
      }
    }

    osMutexRelease(pMutex_);
  }
}

void CDroneCanHandler::ReceiveCanFrames()
{
  CDroneCanHandler& orInstance = GetInstance();

  // Drain all frames currently in the driver's RX FIFO and feed them to libcanard,
  // which reassembles multi-frame transfers and invokes the RX callbacks.
  uint32_t uId;
  bool     bExtended;
  uint8_t  auData[BXCAN_MAX_DLC];
  uint8_t  uDlc;

  while (BxCan_Receive(&uId, &bExtended, auData, &uDlc))
  {
    CanardCANFrame oFrame;
    oFrame.id       = bExtended ? (uId | CANARD_CAN_FRAME_EFF) : uId;
    oFrame.data_len = uDlc;
    oFrame.iface_id = 0U;
    for (uint8_t i = 0U; i < uDlc; ++i)
    {
      oFrame.data[i] = auData[i];
    }

    const uint64_t uTimestampUs = GetMicroseconds();

    // Block for the mutex so a (rare) GetNodeInfo request is never dropped; the
    // sender task holds it only briefly.
    if (osOK == osMutexAcquire(orInstance.pMutex_, osWaitForever))
    {
      canardHandleRxFrame(&orInstance.oCanardInstance_, &oFrame, uTimestampUs);
      osMutexRelease(orInstance.pMutex_);
    }
  }

  // Yield so this polling loop does not starve the rest of the system.
  osDelay(1U);
}

void CDroneCanHandler::onTransferReception_(CanardInstance* opIns, CanardRxTransfer* opTransfer)
{
  // Respond to uavcan.protocol.GetNodeInfo requests so the flight controller and
  // GUI can resolve this node's name, software/hardware version and unique ID.
  if ((opTransfer->transfer_type == CanardTransferTypeRequest) &&
      (opTransfer->data_type_id  == DRONE_CAN_DSDL_PROTOCOL_GET_NODE_INFO_ID))
  {
    // 16-byte hardware unique ID from the STM32 96-bit UID (padded with zeros).
    uint8_t auUniqueId[16];
    memset(auUniqueId, 0, sizeof(auUniqueId));
    const uint32_t uUid0 = HAL_GetUIDw0();
    const uint32_t uUid1 = HAL_GetUIDw1();
    const uint32_t uUid2 = HAL_GetUIDw2();
    memcpy(&auUniqueId[0], &uUid0, sizeof(uUid0));
    memcpy(&auUniqueId[4], &uUid1, sizeof(uUid1));
    memcpy(&auUniqueId[8], &uUid2, sizeof(uUid2));

    uint8_t auPayload[DRONE_CAN_DSDL_GET_NODE_INFO_MAX_PAYLOAD_SIZE];
    const uint16_t uPayloadLen = DroneCanDsdl_SerializeGetNodeInfoResponse(
        auPayload,
        CDroneCanOutputHandler::GetUptimeSec(),
        DRONE_CAN_DSDL_NODE_HEALTH_OK,
        DRONE_CAN_DSDL_NODE_MODE_OPERATIONAL,
        static_cast<uint8_t>(CSoftwareVersion::skuMajor_),
        static_cast<uint8_t>(CSoftwareVersion::skuMinor_),
        /* vcs_commit */ 0U,
        skuHardwareVersionMajor,
        skuHardwareVersionMinor,
        auUniqueId,
        skacNodeName,
        static_cast<uint16_t>(strlen(skacNodeName)));

    // The response must echo the request's transfer ID. This callback runs while
    // ReceiveFrames() already holds the libcanard mutex, so canardRequestOrRespond
    // must be called directly here WITHOUT re-acquiring it (the mutex is not
    // recursive). The queued frames are drained later by TransmitFrames().
    uint8_t uTransferId = opTransfer->transfer_id;
    (void)canardRequestOrRespond(opIns,
                                 opTransfer->source_node_id,
                                 DRONE_CAN_DSDL_PROTOCOL_GET_NODE_INFO_SIGNATURE,
                                 DRONE_CAN_DSDL_PROTOCOL_GET_NODE_INFO_ID,
                                 &uTransferId,
                                 opTransfer->priority,
                                 CanardResponse,
                                 auPayload,
                                 uPayloadLen);
  }
}

bool CDroneCanHandler::shouldAcceptTransfer_(const CanardInstance* kopIns,
                                              uint64_t* oupDataTypeSignature,
                                              uint16_t  uDataTypeId,
                                              CanardTransferType eTransferType,
                                              uint8_t   uSourceNodeId)
{
  (void)kopIns;
  (void)uSourceNodeId;

  // Accept uavcan.protocol.GetNodeInfo service requests; publish-only otherwise.
  if ((eTransferType == CanardTransferTypeRequest) &&
      (uDataTypeId == DRONE_CAN_DSDL_PROTOCOL_GET_NODE_INFO_ID))
  {
    *oupDataTypeSignature = DRONE_CAN_DSDL_PROTOCOL_GET_NODE_INFO_SIGNATURE;
    return true;
  }

  return false;
}

#else
// ISO C/C++ forbid an empty translation unit.
typedef int CDroneCanHandlerNoDroneCan_t;
#endif // USE_DRONECAN
