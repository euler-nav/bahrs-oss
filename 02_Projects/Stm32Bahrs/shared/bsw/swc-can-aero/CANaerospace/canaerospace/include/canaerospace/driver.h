/*
 * CANaerospace driver interface
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_DRIVER_H_
#define CANAEROSPACE_DRIVER_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * CAN ID masks
 * @{
 */
static const uint32_t CANAS_CAN_MASK_STDID = ((uint32_t)0x000007FFu);
static const uint32_t CANAS_CAN_MASK_EXTID = ((uint32_t)0x1FFFFFFFu);
/**
 * @}
 */

/**
 * CAN flags, to be set on CAN ID
 * @{
 */
static const uint32_t CANAS_CAN_FLAG_EFF = (1U << 31);  ///< Extended frame format
static const uint32_t CANAS_CAN_FLAG_RTR = (1U << 30);  ///< Remote transmission request
/**
 * @}
 */

/**
 * CAN frame
 */
typedef struct
{
    uint8_t data[8];
    uint32_t id;      ///< Full ID (Standard + Extended) and flags (CANAS_CAN_FLAG_*)
    uint8_t dlc;      ///< Data length code
} CanasCanFrame;

/**
 * Acceptance filter configuration.
 * Use flags to filter messages by type. @ref CANAS_CAN_FLAG_EFF @ref CANAS_CAN_FLAG_RTR.
 */
typedef struct
{
    uint32_t id;
    uint32_t mask;
} CanasCanFilterConfig;

#ifdef __cplusplus
}
#endif
#endif
