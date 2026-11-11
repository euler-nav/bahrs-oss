/*
 * STM32 CAN driver for the CANaerospace library
 * 
 * Note that this driver can work either under FreeRTOS or without OS.
 * Consider the difference in frame reception API below.
 * 
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CAN_DRIVER_STM32CL_H_
#define CAN_DRIVER_STM32CL_H_

#include <stdint.h>
#include <canaerospace/driver.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Error masks.
 * Consider the relevant API call @ref canYieldErrors.
 */
typedef enum
{
    CAN_ERRFLAG_TX_TIMEOUT         = 1,
    CAN_ERRFLAG_TX_OVERFLOW        = 2,
    CAN_ERRFLAG_RX_OVERFLOW        = 4,
    CAN_ERRFLAG_RX_LOST            = 8,
    CAN_ERRFLAG_HARDWARE           = 16,
    CAN_ERRFLAG_BUS_OFF            = 32
} CanErrorFlag;

/**
 * Number of filters available for each interface.
 */
static const int CAN_FILTERS_PER_IFACE = 14;

/**
 * Number of interfaces available, 1 or 2.
 * May be redefined explicitly.
 */
#define CAN_IFACE_COUNT 1

/**
 * Initializes the driver.
 * @param [in] bitrate         CAN speed, in bits per second
 * @param [in] tx_timeout_usec Frame transmission timeout, in microseconds
 * @return 0 on success, negative on failure.
 */
int canInit(int bitrate, unsigned int tx_timeout_usec);

/**
 * Returns and clears the interface error mask.
 * Mask fields are defined in the enumeration @ref CanErrorFlag.
 * Note that this call performs destructive reading, i.e. error masks will be cleared.
 * You should call this function periodically to check the sanity of the interfaces.
 */
unsigned int canYieldErrors(int iface);

/**
 * Send the frame through the interface
 * @return 1 on success, 0 if there is no free space in buffer, negative on error.
 */
int canSend(int iface, const CanasCanFrame* pframe);

/**
 * Setup CAN filters for the specified interface.
 * Number of filters must not be higher than @ref CAN_FILTERS_PER_IFACE.
 * @return 0 on success, negative on error.
 */
int canFilterSetup(int iface, const CanasCanFilterConfig* pfilters, int filters_len);

#if CAN_FREERTOS || CAN_CHIBIOS
/**
 * Reads from any interface and returns its index through pointer.
 * This function will block for up to timeout_usec microseconds.
 * @note This function allows simultaneous calls to other API functions while blocked. Consider the examples.
 * @param [out] piface       Pointer to the interface index
 * @param [out] pframe       Pointer to the received frame
 * @param [in]  timeout_usec Block timeout in microseconds
 * @return 1 on successful read, 0 on timeout, -1 on error.
 */
int canReceive(int* piface, CanasCanFrame* pframe, unsigned int timeout_usec);
#else
/**
 * Reads from any interface and returns its index through pointer.
 * This function will never block.
 * @param [out] piface Pointer to the interface index
 * @param [out] pframe Pointer to the frame received
 * @return 1 on successful read, 0 if there are no frames available, -1 on error.
 */
int canReceive(int* piface, CanasCanFrame* pframe);
#endif

#ifdef __cplusplus
}
#endif
#endif
