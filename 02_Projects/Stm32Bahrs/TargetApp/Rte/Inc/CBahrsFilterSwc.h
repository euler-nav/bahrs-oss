/**
 * @file CBahrsFilerSwc.h
 * @brief Implementation of the BAHRS filter software component
 * @author Fedor Baklanov
 * @date 08 June 2022
 */

#ifndef C_BAHRS_FILTER_SWC_H
#define C_BAHRS_FILTER_SWC_H

#include "General/CSoftwareComponentBase.h"
#include "BahrsFilterApi.h"

#ifndef _MSC_VER
#include "cmsis_os.h"
#endif /* _MSC_VER */

class CBahrsFilterSwc : public CSoftwareComponent<CBahrsFilterSwc, 1U>
{
  friend class CSoftwareComponent<CBahrsFilterSwc, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CBahrsFilterSwc)
  DECLARE_MANDATORY_APIS(CBahrsFilterSwc)

public:
  /**
   * @brief A wrapper around CBahrs::SetInput(). Has mutex protection.
   * @param korImuData Reference to input IMU data.
  */
  void SetImuInput();

  /**
   * @brief A wrapper around CBahrs::SetInput(). Has mutex protection.
   * @param korPressureData Reference to input pressure data.
  */
  void SetPressureInput();

  /**
   * @brief A wrapper arounf CBahrs::Step().
   * @param uTimestampUs Time of the API call in microseconds.
  */
  void Step(uint64_t uTimestampUs);

protected:

private:
  /**
   * @brief The constructor creates a mutex for shared BAHRS filter data.
  */
  CBahrsFilterSwc();
  ~CBahrsFilterSwc() = default;

#ifndef _MSC_VER
  /**
   * The typedef copied from a CMSIS_OS header.
   */
  typedef StaticSemaphore_t osStaticMutexDef_t;

  osMutexId_t pMutexHandle_; ///< Mutex handle.
  osStaticMutexDef_t sMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t sMutexAttributes_ { NULL, 0, &sMutexControlBlock_, sizeof(sMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.
#endif /* _MSC_VER */
};

#endif /* C_BAHRS_FILTER_SWC_H */
