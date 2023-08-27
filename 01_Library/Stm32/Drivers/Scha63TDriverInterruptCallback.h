/**
 * @file Scha63TDriverInterruptCallbacks.h
 * @brief Declaration of SCHA63T driver callbacks.
 * @author Fedor Baklanov
 * @date 01 April 2023
 */

#ifndef SCHA63T_DRIVER_INTERRUPT_CALLBACK_H
#define SCHA63T_DRIVER_INTERRUPT_CALLBACK_H

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * \brief Poll the SCHA63T from the high-frequency interrupt.
 */
void PollScha63T();

#ifdef __cplusplus
}
#endif

#endif /* SCHA63T_DRIVER_INTERRUPT_CALLBACK_H */
