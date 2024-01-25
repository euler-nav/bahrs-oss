/**
 * @file CImuMonitorSwc.h
 * @brief Implementation of the IMU monitor software component
 * @author Fedor Baklanov
 * @date 6 December 2023
 * @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef C_IMU_MONITOR_SWC_H
#define C_IMU_MONITOR_SWC_H

#include "General/CSoftwareComponentBase.h"
#include "ImuMonitorApi.h"

#ifndef _MSC_VER
#include "cmsis_os.h"
#endif /* _MSC_VER */

class CImuMonitorSwc : public CSoftwareComponent<CImuMonitorSwc, 1U>
{
  friend class CSoftwareComponent<CImuMonitorSwc, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CImuMonitorSwc)
  DECLARE_MANDATORY_APIS(CImuMonitorSwc)

public:
  /**
   * @brief Run IMU monitor.
  */
  void Run();

protected:

private:
  CImuMonitorSwc() = default;
  ~CImuMonitorSwc() = default;
};

#endif /* C_IMU_MONITOR_SWC_H */
