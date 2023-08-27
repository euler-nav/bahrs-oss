/**
 * @file CSyncPulseHandler.h
 * @brief Declaration of the CSyncPulseHandler software component.
 * @author Fedor Baklanov
 * @date 31 May 2023
 */

#ifndef C_SYNC_PULSE_HANDLER_H
#define C_SYNC_PULSE_HANDLER_H

#include "General/CSoftwareComponentBase.h"

class CSyncPulseHandler : public CSoftwareComponent<CSyncPulseHandler, 0U>
{
  friend class CSoftwareComponent<CSyncPulseHandler, 0U>;
  FORBID_CLASS_COPY_AND_MOVE(CSyncPulseHandler)
  DECLARE_MANDATORY_APIS(CSyncPulseHandler)

public:
  /**
   * A routine called in the dedicated interrupt callback.
   */
  static void SyncPulseCallback();

  /**
   * Pulse processor. Called from the dedicated FreeRTOS task.
   * \param uTimestampUs Timestamp of the interrupt.
   */
  static void ProcessSyncPulse(uint64_t uTimestampUs);

protected:

private:
  CSyncPulseHandler() = default;
  ~CSyncPulseHandler() = default;
};

#endif /* C_SYNC_PULSE_HANDLER_H */
