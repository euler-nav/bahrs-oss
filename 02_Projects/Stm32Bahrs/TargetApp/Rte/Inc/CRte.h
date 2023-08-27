/**
 * @file CRte.h
 * @brief Declaration of the runtime environment class.
 * @author Fedor Baklanov
 * @date 07 June 2022
 */

#ifndef C_RTE_H
#define C_RTE_H

#include "CSoftwareComponentPort.h"
#include "CBahrsFilterSwc.h"
#include "RteTypes.h"

// The code below is required for target firmware only
#ifndef _MSC_VER
#include "CBmp384Driver.h"
#include "CScha63TDriver.h"
#include "Scha63TDriverInterruptCallback.h"
#include "CMmc5983Driver.h"
#include "CRs232OutputHandler.h"
#include "CBmm150Driver.h"
#include "CIcm20789Driver.h"
#include "CSyncPulseHandler.h"
#endif /* _MSC_VER */

/**
 * Runtime environment (RTE) class. The class is a singleton. It collects all the application application
 * software components and ports. RTE shall be created during initialization.
 */
class CRte
{
public:
  CRte(const CRte &orOther) = delete;
  CRte(CRte &&orOther) = delete;
  CRte& operator=(const CRte &orOther) = delete;
  CRte& operator=(CRte &&orOther) = delete;

  friend class CBahrsFilterSwc;
  friend class CBmm150Driver;
  friend class CBmp384Driver;
  friend class CScha63TDriver;
  friend class CIcm20789Driver;
  friend class CMmc5983Driver;
  friend class CRs232OutputHandler;
  friend class CCanOutputHandler;
  friend class CSyncPulseHandler;
  friend class COutputTransformer;
  friend class CMagnetometerCompensator;
  friend class CMagneticHeadingFilterSwc;

  enum class EPortIds : uint8_t
  {
    eInvalid = 0U,
    ePortScha63TInput,
    ePortBmp384Input,
    ePortMmc5983nput,
    ePortBmm150Input1,
    ePortBmm150Input2,
    ePortBahrsFilterOutput,
    ePortIcm20789BaroInput1,
    ePortIcm20789ImuInput1,
    ePortIcm20789BaroInput2,
    ePortIcm20789ImuInput2,
    ePortSyncPulseTime
  };

  /**
   * Get a pointer to the RTE instance.
   */
  static CRte& GetInstance();

  /**
   * Initialize the RTE. Creates and initializes all the member objects.
   */
  void Init();

protected:

private:
  /**
   * The default constructor. Called only by the CRte::GetInstance().
   */
  CRte();
  ~CRte() = default;

  //
  // Declaration of ports
  //

  CSoftwareComponentPort<SImuDataScha63T, static_cast<uint8_t>(EPortIds::ePortScha63TInput)> oScha63TDriverPort_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePortBmp384Input)> oPortBmp384Input_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::ePortMmc5983nput)> oPortMmc5983Input_;
  CSoftwareComponentPort<SMagneticData, static_cast<uint8_t>(EPortIds::ePortBmm150Input1)> oPortBmm150Input1_;
  CSoftwareComponentPort<SMagneticData, static_cast<uint8_t>(EPortIds::ePortBmm150Input2)> oPortBmm150Input2_;
  CSoftwareComponentPort<NBahrsFilterApi::SOutputData, static_cast<uint8_t>(EPortIds::ePortBahrsFilterOutput)> oPortBahrsFilterOutput_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePortIcm20789BaroInput1)> oPortIcm20789BaroInput1_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::ePortIcm20789ImuInput1)> oPortIcm20789ImuInput1_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePortIcm20789BaroInput2)> oPortIcm20789BaroInput2_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::ePortIcm20789ImuInput2)> oPortIcm20789ImuInput2_;
  CSoftwareComponentPort<uint64_t, static_cast<uint8_t>(EPortIds::ePortSyncPulseTime)> oPortSyncPulseTime_;

};

#endif /* C_RTE_H */
