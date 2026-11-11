/*
 * STM32 CAN driver for the CANaerospace library
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#if CAN_FREERTOS
#   include <FreeRTOS.h>
#   include <queue.h>
#endif

#if CAN_CHIBIOS
#   include <ch.h>
#   include <hal.h>
#endif

#include "can_driver.h"
#include "internal.h"

#define GLUE2_(A, B) A##B
#define GLUE2(A, B) GLUE2_(A, B)

// macro instantiation:
#define CANx                GLUE2(CAN, CAN_INTERFACE_NUMBER)
#define CANx_Handler        GLUE2(hcan,CAN_INTERFACE_NUMBER)

extern CAN_HandleTypeDef CANx_Handler;
CAN_HandleTypeDef *hcan = &CANx_Handler;

static unsigned int _error_mask[2] = {0, 0};
static unsigned int _frame_tx_timeout_usec = 0;

typedef struct
{
    CanasCanFrame frame;
    uint8_t iface;
} FifoEntry;

typedef struct
{
    FifoEntry* const pbuf;
    const int bufsize;
    int in;
    int out;
    int len;
} Fifo;

#if CAN_IFACE_COUNT == 1
#   define CAN_RX_QUEUE_LEN_TOTAL CAN_RX_QUEUE_LEN
#elif CAN_IFACE_COUNT == 2
#   define CAN_RX_QUEUE_LEN_TOTAL (CAN_RX_QUEUE_LEN * 2)
#else
#   error "CAN_IFACE_COUNT must be 1 or 2"
#endif

#if CAN_FREERTOS

static xQueueHandle _queue_rx = NULL;

#else

static FifoEntry _buff_rx[CAN_RX_QUEUE_LEN_TOTAL];
static Fifo _fifo_rx = { _buff_rx, CAN_RX_QUEUE_LEN_TOTAL, 0, 0, 0 };

#endif

#if CAN_CHIBIOS
static EVENTSOURCE_DECL(_on_rx);
#endif

static FifoEntry _buff_tx[CAN_IFACE_COUNT][CAN_TX_QUEUE_LEN];
static Fifo _fifo_tx[CAN_IFACE_COUNT] =
{
    { _buff_tx[0], CAN_TX_QUEUE_LEN, 0, 0, 0 }
#if CAN_IFACE_COUNT > 1
    ,
    { _buff_tx[1], CAN_TX_QUEUE_LEN, 0, 0, 0 }
#endif
};

/* Software FIFO */

static inline void _fifoPush(Fifo* pfifo, const FifoEntry* pframe, bool* overflow)
{
    pfifo->pbuf[pfifo->in++] = *pframe;
    if (pfifo->in >= pfifo->bufsize)
        pfifo->in = 0;
    if (pfifo->len >= pfifo->bufsize)
    {
        *overflow = true;
        pfifo->out++;
        if (pfifo->out >= pfifo->bufsize)
            pfifo->out = 0;
    }
    else
    {
        *overflow = false;
        pfifo->len++;
    }
}

static inline int _fifoPop(Fifo* pfifo, FifoEntry* pframe)
{
    if (pfifo->len <= 0)
        return 0;
    pfifo->len--;
    if (pframe)
        *pframe = pfifo->pbuf[pfifo->out++];
    if (pfifo->out >= pfifo->bufsize)
        pfifo->out = 0;
    return 1;
}

static inline void _frameSpl2Canas(const CAN_RxHeaderTypeDef* pspl, uint8_t* data, CanasCanFrame* pcanas)
{
    memset(pcanas, 0, sizeof(*pcanas));
    memcpy(pcanas->data, data, pspl->DLC);
    pcanas->dlc = pspl->DLC;
    if (pspl->IDE == CAN_ID_STD)
    {
        pcanas->id = pspl->StdId & CANAS_CAN_MASK_STDID;
    }
    else
    {
        pcanas->id = pspl->ExtId & CANAS_CAN_MASK_EXTID;
        pcanas->id |= CANAS_CAN_FLAG_EFF;
    }
    if (pspl->RTR == CAN_RTR_REMOTE)
        pcanas->id |= CANAS_CAN_FLAG_RTR;
}

static inline void _frameCanas2Spl(const CanasCanFrame* pcanas, CAN_TxHeaderTypeDef* pspl)
{
    memset(pspl, 0, sizeof(*pspl));
    pspl->DLC = pcanas->dlc;
    if (pcanas->id & CANAS_CAN_FLAG_EFF)
    {
        pspl->ExtId = pcanas->id & CANAS_CAN_MASK_EXTID;
        pspl->IDE = CAN_ID_EXT;
    }
    else
    {
        pspl->StdId = pcanas->id & CANAS_CAN_MASK_STDID;
        pspl->IDE = CAN_ID_STD;
    }
    pspl->RTR = (pcanas->id & CANAS_CAN_FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;
}

/* Error monitoring */

static inline void _setErrors(int iface, unsigned int mask)
{
    if (iface == 0 || iface == 1)
        _error_mask[iface] |= mask;
}

static void _pollHardwareErrors(int iface)
{
    unsigned int errmask = 0;

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_BOF))   // There is no way to clear this flag by software
        errmask |= CAN_ERRFLAG_BUS_OFF;

    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_EWG))
    {
        errmask |= CAN_ERRFLAG_HARDWARE;
    }
    if (__HAL_CAN_GET_FLAG(hcan, CAN_FLAG_EPV))
    {
        errmask |= CAN_ERRFLAG_HARDWARE;
    }
    // RX FIFO overflow flag doesn't work - it's always cleared when an overflow occurs
    if (CANx->RF0R & CAN_RF0R_FOVR0 || CANx->RF1R & CAN_RF1R_FOVR1)
    {
        errmask |= CAN_ERRFLAG_RX_LOST;
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV0);
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_FOV1);
    }
    __disable_irq();
    _setErrors(iface, errmask);
    __enable_irq();
}

/* Hardware control */

#define NUM_OF_MAILBOXES 3

#define HAS_EMPTY_MAILBOX(CANx) (!!((CANx)->TSR & CAN_TSR_TME))

#define HAS_PENDING_MAILBOX(CANx) \
    (!(((CANx)->TSR & CAN_TSR_TME0) && ((CANx)->TSR & CAN_TSR_TME1) && ((CANx)->TSR & CAN_TSR_TME2)))

//static inline void _tryTransmit(int iface) __attribute__((always_inline));
static inline void _tryTransmit(int iface)
{
	CAN_TxHeaderTypeDef hdr_spl;
    FifoEntry fe;
    uint32_t TxMailbox;
    if (!HAS_EMPTY_MAILBOX(CANx))
        return;
    if (_fifoPop(&_fifo_tx[iface], &fe))
    {
        _frameCanas2Spl(&fe.frame, &hdr_spl);
        HAL_CAN_AddTxMessage(hcan, &hdr_spl, fe.frame.data,&TxMailbox);
        canTimerSet(iface, _frame_tx_timeout_usec * NUM_OF_MAILBOXES);
    }
}

static inline void _abortPendingTransmissions(int iface)
{
    ((iface == 0) ? CAN1 : CAN2)->TSR |= CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
}

void canTimerIrq(int iface)
{
    // It's not necessary to perform further actions here, because we will get TX IRQ after that.
    _abortPendingTransmissions(iface);
    _setErrors(iface, CAN_ERRFLAG_TX_TIMEOUT);
}

/******************************************************************
  * --------------------- TRANSMISSION ----------------------------
  */
static inline void _genericTxIrqHandler(int iface) __attribute__((always_inline));
static inline void _genericTxIrqHandler(int iface)
{
	// One of the Mailboxes got free. Check if there is new data to be transmitted
	_tryTransmit(iface);

	if (!HAS_PENDING_MAILBOX(CANx))    // Nothing left to transmit, so timer is no use anymore
	  canTimerStop(iface);
}

/**
  * @brief  Transmission Mailbox 0 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1 )
	{
		_genericTxIrqHandler(0);
	}
	if( hcan->Instance == CAN2 )
	{
		_genericTxIrqHandler(0);
	}
}

/**
  * @brief  Transmission Mailbox 1 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1 )
	{
		_genericTxIrqHandler(0);
	}
	if( hcan->Instance == CAN2 )
	{
		_genericTxIrqHandler(0);
	}
}

/**
  * @brief  Transmission Mailbox 2 complete callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1 )
	{
		_genericTxIrqHandler(0);
	}
	if( hcan->Instance == CAN2 )
	{
		_genericTxIrqHandler(0);
	}
}

/******************************************************************
  * ----------------------- RECEPTION -----------------------------
  */
static inline void _genericRxIrqHandler(uint32_t hwfifo, int iface)
{
#if CAN_FREERTOS
    portBASE_TYPE task_woken = 0;
#endif
    CAN_RxHeaderTypeDef hdr_spl;
    uint8_t data_spl[8];

	HAL_CAN_GetRxMessage(hcan, hwfifo, &hdr_spl, data_spl);

	FifoEntry fifo_entry;
	fifo_entry.iface = iface;
	_frameSpl2Canas(&hdr_spl, data_spl, &fifo_entry.frame);

	bool overflow = 0;
#if CAN_FREERTOS
	if (xQueueIsQueueFullFromISR(_queue_rx))
	{
		FifoEntry waste;
		xQueueReceiveFromISR(_queue_rx, &waste, &task_woken);  // Queue is full, so we have to kill the last entry
		overflow = true;
	}
	task_woken = 0;
	xQueueSendFromISR(_queue_rx, &fifo_entry, &task_woken);
#else
	_fifoPush(&_fifo_rx, &fifo_entry, &overflow);
#endif
#if CAN_CHIBIOS
	chSysLockFromIsr();
	chEvtBroadcastFlagsI(&_on_rx, ALL_EVENTS);
	chSysUnlockFromIsr();
#endif
	if (overflow)
		_setErrors(iface, CAN_ERRFLAG_RX_OVERFLOW);
#if CAN_FREERTOS
    portEND_SWITCHING_ISR(task_woken);
#endif
}

/**
  * @brief  Rx FIFO 0 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1 )
	{
		_genericRxIrqHandler(CAN_RX_FIFO0, 0);
	}
	if( hcan->Instance == CAN2 )
	{
		_genericRxIrqHandler(CAN_RX_FIFO0, 0);
	}
}

/**
  * @brief  Rx FIFO 1 message pending callback.
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if( hcan->Instance == CAN1 )
	{
		_genericRxIrqHandler(CAN_RX_FIFO1, 0);
	}
	if( hcan->Instance == CAN2 )
	{
		_genericRxIrqHandler(CAN_RX_FIFO1, 0);
	}
}

static void _initInterrupts(void)
{
	uint32_t ActiveITs = CAN_IT_TX_MAILBOX_EMPTY |
			             CAN_IT_RX_FIFO0_MSG_PENDING |
                         CAN_IT_RX_FIFO1_MSG_PENDING ;
	HAL_CAN_ActivateNotification(hcan, ActiveITs);
}

static int _configureTimings(int target_bitrate, CAN_InitTypeDef* output)
{
    /*
     *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))
     *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))
     * let:
     *   BS = 1 + BS1 + BS2
     *   PRESCALER_BS = PRESCALER * BS
     * ==>
     *   PRESCALER_BS = PCLK / BITRATE
     */
    if (target_bitrate < 20000 || target_bitrate > 2000000)
        return -1;

    // Done in CubeMX. Nothing to be done here
    return 0;
}

static int _leave_error(int result)
{
    HAL_CAN_MspDeInit(hcan);
    canTimerDeinit();
    return (result < 0) ? result : -1;
}

/* Interface */
int canInit(int bitrate, unsigned int tx_timeout_usec)
{
    int result = -1;

    if (tx_timeout_usec <= 125)    // Min message time for 1Mbps, std id.
        return -1;
    _frame_tx_timeout_usec = tx_timeout_usec;

    /*
     * Enable the CAN cells and the timer; selftest
     */
    if (canTimerInit() != 0)
    {
        return _leave_error(result);
    }

#if 0
    if (canSelftest(CANx) != 0)
    {
        return _leave_error();
    }
#endif

    __HAL_CAN_RESET_HANDLE_STATE(hcan);

    /*
     * CAN configuration (same parameters for all interfaces)
     */
    hcan->Instance                  = CANx;
    hcan->Init.Prescaler            = 18;
    hcan->Init.Mode                 = CAN_MODE_NORMAL;
    hcan->Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan->Init.TimeSeg1             = CAN_BS1_2TQ;
    hcan->Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan->Init.AutoBusOff           = ENABLE;    // Automatic Bus-Off recovery
    hcan->Init.AutoWakeUp           = ENABLE;
    hcan->Init.TimeTriggeredMode    = DISABLE;
    hcan->Init.AutoRetransmission   = DISABLE;
    hcan->Init.ReceiveFifoLocked    = DISABLE;
    hcan->Init.TransmitFifoPriority = ENABLE;    // Transmit in order
    hcan->Init.Mode                 = CAN_MODE_NORMAL;

    result = _configureTimings(bitrate, &hcan->Init);
    if (result != 0)
    {
        return _leave_error(result);
    }

    if (HAL_CAN_Init(hcan) != HAL_OK)
    {
        return _leave_error(result);
    }

    _initInterrupts();
    result = canFilterInit();
    if (result != 0)
    {
        return _leave_error(result);
    }

    /*
     * OS-specific initialization
     */
#if CAN_FREERTOS
    if (_queue_rx == NULL)
    {
        _queue_rx = xQueueCreate(CAN_RX_QUEUE_LEN_TOTAL, sizeof(FifoEntry));
    }
    if (_queue_rx == NULL)
    {
        return _leave_error(result);
    }
#endif
#if CAN_CHIBIOS
    chEvtInit(&_on_rx);
#endif

    /* We are good to start CAN */
    if (HAL_CAN_Start(hcan) != HAL_OK)
    {
        return _leave_error(result);
    }

    return 0;
}

unsigned int canYieldErrors(int iface)
{
    if (iface < 0 || iface >= CAN_IFACE_COUNT)
        return -1;

    _pollHardwareErrors(iface);

    __disable_irq();
    const unsigned int errmask = _error_mask[iface];
    _error_mask[iface] = 0;
    __enable_irq();

    return errmask;
}

int canSend(int iface, const CanasCanFrame* pframe)
{
    if ((iface < 0 || iface >= CAN_IFACE_COUNT) || pframe == NULL)
        return -1;

    FifoEntry fe;
    fe.iface = -1;   // Not used really
    fe.frame = *pframe;

    taskENTER_CRITICAL();
    bool overflow = false;
    _fifoPush(&_fifo_tx[iface], &fe, &overflow);
    _tryTransmit(iface);                          // Will transmit if there is free transmit mailbox
    if (overflow)
    {
        _setErrors(iface, CAN_ERRFLAG_TX_OVERFLOW);
        _abortPendingTransmissions(iface);        // Objective is to transmit the newer data faster
    }
    taskEXIT_CRITICAL();
    return 1;
}

#if CAN_FREERTOS

int canReceive(int* piface, CanasCanFrame* pframe, unsigned int timeout_usec)
{
    if (piface == NULL || pframe == NULL)
        return -1;
    FifoEntry fe;
    const unsigned int timeout_ticks = timeout_usec / (1000000 / configTICK_RATE_HZ);
    if (xQueueReceive(_queue_rx, &fe, timeout_ticks))
    {
        *pframe = fe.frame;
        *piface = fe.iface;
        return 1;
    }
    return 0;    // timed out
}

#else

static int _tryReceive(int* piface, CanasCanFrame* pframe)
{
    FifoEntry fe;
    __disable_irq();
    const int retval = _fifoPop(&_fifo_rx, &fe);
    __enable_irq();
    if (retval > 0)
    {
        *pframe = fe.frame;
        *piface = fe.iface;
    }
    return retval;
}

#if CAN_CHIBIOS

int canReceive(int* piface, CanasCanFrame* pframe, unsigned int timeout_usec)
{
    if (piface == NULL || pframe == NULL)
        return -1;

    EventListener listener;
    chEvtRegisterMask(&_on_rx, &listener, ALL_EVENTS);

    int res = _tryReceive(piface, pframe);
    if (!res)
    {
        chEvtWaitAnyTimeout(ALL_EVENTS, US2ST(timeout_usec));
        res = _tryReceive(piface, pframe);
    }
    chEvtUnregister(&_on_rx, &listener);
    return res;  // 0 or 1
}

#else

int canReceive(int* piface, CanasCanFrame* pframe)
{
    if (piface == NULL || pframe == NULL)
        return -1;
    return _tryReceive(piface, pframe);
}

#endif // CAN_CHIBIOS
#endif // CAN_FREERTOS
