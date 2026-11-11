/// @file bxcan.c
/// @brief Implementation of minimal STM32 bxCAN driver for the DroneCAN stack (see bxcan.h).
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

// The DroneCAN stack is a build variant: this translation unit is empty
// unless USE_DRONECAN is defined.
#ifdef USE_DRONECAN

#include "bxcan.h"
#include "stm32f4xx_hal.h"

// CAN instance selection.
//
// CAN_INTERFACE_NUMBER is provided by the build (1 on HW_V3 -> CAN1, defaults to
// 2 on HW_V2 -> CAN2). The CubeMX-generated handle (hcan1/hcan2) is used so that
// HAL_CAN_Init() invokes the project's HAL_CAN_MspInit() -- GPIO, clocks and NVIC.
//
// bxCAN filter banks are shared between CAN1 (master, banks [0, CAN2SB-1]) and
// CAN2 (slave, banks [CAN2SB, 27]). The accept-all bank must belong to the
// instance in use, hence the per-instance bank / split-point below.
#ifndef CAN_INTERFACE_NUMBER
#  define CAN_INTERFACE_NUMBER 2
#endif

#if (CAN_INTERFACE_NUMBER == 2)
extern CAN_HandleTypeDef hcan2;
#  define BXCAN_HANDLE            (&hcan2)
#  define BXCAN_INSTANCE          CAN2
#  define BXCAN_FILTER_BANK       0U   // CAN2SB == 0 -> CAN2 owns all banks
#  define BXCAN_SLAVE_START_BANK  0U
#else
extern CAN_HandleTypeDef hcan1;
#  define BXCAN_HANDLE            (&hcan1)
#  define BXCAN_INSTANCE          CAN1
#  define BXCAN_FILTER_BANK       0U   // CAN1 owns banks [0, CAN2SB-1]
#  define BXCAN_SLAVE_START_BANK  14U
#endif

/// @brief 29-bit extended identifier mask.
#define BXCAN_EXT_ID_MASK 0x1FFFFFFFU

bool BxCan_Init(void)
{
    CAN_HandleTypeDef* const pCan = BXCAN_HANDLE;

    // 45 MHz APB1 / (Prescaler 18 * (1 + BS1 2TQ + BS2 2TQ) = 5 TQ) = 500 kbit/s.
    pCan->Instance                  = BXCAN_INSTANCE;
    pCan->Init.Prescaler            = 18U;
    pCan->Init.Mode                 = CAN_MODE_NORMAL;
    pCan->Init.SyncJumpWidth        = CAN_SJW_1TQ;
    pCan->Init.TimeSeg1             = CAN_BS1_2TQ;
    pCan->Init.TimeSeg2             = CAN_BS2_2TQ;
    pCan->Init.TimeTriggeredMode    = DISABLE;
    pCan->Init.AutoBusOff           = ENABLE;
    pCan->Init.AutoWakeUp           = ENABLE;
    pCan->Init.AutoRetransmission   = DISABLE; // drop un-ACKed frames instead of blocking a mailbox
    pCan->Init.ReceiveFifoLocked    = DISABLE;
    pCan->Init.TransmitFifoPriority = ENABLE;

    if (HAL_CAN_Init(pCan) != HAL_OK)
    {
        return false;
    }

    // Accept every identifier into RX FIFO0. DroneCAN filters by node ID / data
    // type in software, so no hardware identifier filtering is required.
    CAN_FilterTypeDef oFilter;
    oFilter.FilterActivation     = CAN_FILTER_ENABLE;
    oFilter.FilterBank           = BXCAN_FILTER_BANK;
    oFilter.FilterMode           = CAN_FILTERMODE_IDMASK;
    oFilter.FilterScale          = CAN_FILTERSCALE_32BIT;
    oFilter.FilterIdHigh         = 0x0000U;
    oFilter.FilterIdLow          = 0x0000U;
    oFilter.FilterMaskIdHigh     = 0x0000U; // mask 0 -> every identifier bit is "don't care"
    oFilter.FilterMaskIdLow      = 0x0000U;
    oFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    oFilter.SlaveStartFilterBank = BXCAN_SLAVE_START_BANK;

    if (HAL_CAN_ConfigFilter(pCan, &oFilter) != HAL_OK)
    {
        return false;
    }

    // No notifications are activated: transmission and reception are polled.
    return (HAL_CAN_Start(pCan) == HAL_OK);
}

bool BxCan_Send(uint32_t uExtId, const uint8_t* kpData, uint8_t uDlc)
{
    CAN_HandleTypeDef* const pCan = BXCAN_HANDLE;

    if (HAL_CAN_GetTxMailboxesFreeLevel(pCan) == 0U)
    {
        return false; // all mailboxes busy -- the caller retries on the next cycle
    }

    CAN_TxHeaderTypeDef oHeader;
    oHeader.ExtId              = uExtId & BXCAN_EXT_ID_MASK;
    oHeader.StdId              = 0U;
    oHeader.IDE                = CAN_ID_EXT;
    oHeader.RTR                = CAN_RTR_DATA;
    oHeader.DLC                = (uDlc > BXCAN_MAX_DLC) ? BXCAN_MAX_DLC : uDlc;
    oHeader.TransmitGlobalTime = DISABLE;

    uint32_t uMailbox;
    return (HAL_CAN_AddTxMessage(pCan, &oHeader, (uint8_t*)kpData, &uMailbox) == HAL_OK);
}

bool BxCan_Receive(uint32_t* opuId, bool* opbExtended, uint8_t* opuData, uint8_t* opuDlc)
{
    CAN_HandleTypeDef* const pCan = BXCAN_HANDLE;

    if (HAL_CAN_GetRxFifoFillLevel(pCan, CAN_RX_FIFO0) == 0U)
    {
        return false;
    }

    CAN_RxHeaderTypeDef oHeader;
    if (HAL_CAN_GetRxMessage(pCan, CAN_RX_FIFO0, &oHeader, opuData) != HAL_OK)
    {
        return false;
    }

    if (oHeader.IDE == CAN_ID_EXT)
    {
        *opuId       = oHeader.ExtId;
        *opbExtended = true;
    }
    else
    {
        *opuId       = oHeader.StdId;
        *opbExtended = false;
    }
    *opuDlc = (uint8_t)oHeader.DLC;
    return true;
}

#else
// ISO C/C++ forbid an empty translation unit.
typedef int BxCanNoDroneCan_t;
#endif // USE_DRONECAN
