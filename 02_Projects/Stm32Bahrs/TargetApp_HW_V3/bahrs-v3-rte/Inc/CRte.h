/**
 * @file CRte.h
 * @brief Declaration of the runtime environment class.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date 29.05.2026 at 10:43:47
 */

#ifndef C_RTE_H
#define C_RTE_H

#include "CSoftwareComponentPort.h"
#include "RteTypesGenerated.h"
#include "RteTypesUserDefined.h"
#include "General/HelperSoftwareComponentMacros.h"

#include "COutputTransformer.h"
#include "CPressureCompensator.h"
#include "CImuMonitorSwc.h"
#include "CBaroMonitorSwc.h"
#include "CBahrsFilterSwc.h"
#include "CAttitudeMonitorSwc.h"
#include "CVerticalChannelMonitorSwc.h"
#ifndef _MSC_VER
#include "CBmi270Driver.h"
#include "CAsm330lhhDriver.h"
#include "CRs232OutputHandler.h"
#include "CSpiHandler.h"
#include "CI2CHandler.h"
#include "CBmp384Driver.h"
#include "CLps22hhDriver.h"
#include "CScha63TDriver.h"
#include "CIcp20100Driver.h"
#include "CMmc5983Driver.h"
#include "CLis3mdlDriver.h"
#include "CBmm350Driver.h"
#include "CSyncPulseHandler.h"
#endif /* _MSC_VER */


/**
 * Runtime environment (RTE) class. The class is a singleton. It collects all the application application
 * software components and ports. RTE shall be created during initialization.
 */
class CRte
{
public:
  FORBID_CLASS_COPY_AND_MOVE(CRte)

  friend class COutputTransformer;
  friend class CPressureCompensator;
  friend class CImuMonitorSwc;
  friend class CBaroMonitorSwc;
  friend class CBahrsFilterSwc;
  friend class CAttitudeMonitorSwc;
  friend class CVerticalChannelMonitorSwc;
#ifndef _MSC_VER
  friend class CBmi270Driver;
  friend class CAsm330lhhDriver;
  friend class CRs232OutputHandler;
  friend class CSpiHandler;
  friend class CI2CHandler;
  friend class CBmp384Driver;
  friend class CLps22hhDriver;
  friend class CScha63TDriver;
  friend class CIcp20100Driver;
  friend class CMmc5983Driver;
  friend class CLis3mdlDriver;
  friend class CBmm350Driver;
  friend class CSyncPulseHandler;
#endif /* _MSC_VER */


  enum class EPortIds : uint8_t
  {
    eImuInput1,
    eImuInput2,
    eImuInput3,
    ePressureInput1,
    ePressureInput2,
    ePressureInput3,
    eCompensatedPressureInput1,
    eCompensatedPressureInput2,
    eCompensatedPressureInput3,
    eMagnetometerInput1,
    eMagnetometerInput2,
    eMagnetometerInput3,
    eSyncPulseTime,
    eImuOutput,
    eCompensatedMagnetometerData,
    eMagneticHeading,
    eCompensatedMagnetometerDataInVehicleFrame,
    eImuDataAfterMonitor,
    eSafePressureData1,
    eSafePressureData2,
    eSafePressureData3,
    eBahrsFilterOutput1,
    eBahrsFilterOutput2,
    eBahrsFilterOutput3,
    eVehicleAttitude1,
    eVehicleAttitude2,
    eVehicleAttitude3,
    eSafeVehicleAttitude,
    eSafeVerticalChannelData
  };


  enum class ERunnableIds : uint8_t
  {
    eInvalid = 0U,
    eRunnableBahrsFilterStep1,
    eRunnableBahrsFilterStep2,
    eRunnableBahrsFilterStep3,
    eRunnableBahrsFilterSetImuInput1,
    eRunnableBahrsFilterSetImuInput2,
    eRunnableBahrsFilterSetImuInput3,
    eRunnableBahrsFilterSetPressureInput1,
    eRunnableBahrsFilterSetPressureInput2,
    eRunnableBahrsFilterSetPressureInput3,
    eRunnableMagneticHeadingFilterStep1,
    eRunnableMagneticHeadingFilterStep2,
    eRunnableMagneticHeadingFilterStep3,
    eRunnableMagneticHeadingFilterSetAttitude1,
    eRunnableMagneticHeadingFilterSetAttitude2,
    eRunnableMagneticHeadingFilterSetAttitude3,
    eRunnableImuMonitorRun,
    eRunnableBarometerMonitorRun,
    eRunnableAttitudeMonitorRun,
    eRunnableVerticalChannelMonitorRun
  };
  
#ifdef _MSC_VER
  friend void WriteToRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes);
  friend void handleRunnableCallEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile);
  friend void WriteToUserDefinedRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes);
  friend void SetPortsToRecordInReplayTool();
#endif // _MSC_VER

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

  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInput1)> oPortImuInput1_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInput2)> oPortImuInput2_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInput3)> oPortImuInput3_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePressureInput1)> oPortPressureInput1_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePressureInput2)> oPortPressureInput2_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePressureInput3)> oPortPressureInput3_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedPressureInput1)> oPortCompensatedPressureInput1_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedPressureInput2)> oPortCompensatedPressureInput2_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedPressureInput3)> oPortCompensatedPressureInput3_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eMagnetometerInput1)> oPortMagnetometerInput1_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eMagnetometerInput2)> oPortMagnetometerInput2_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eMagnetometerInput3)> oPortMagnetometerInput3_;
  CSoftwareComponentPort<STimeOfSyncPulse, static_cast<uint8_t>(EPortIds::eSyncPulseTime)> oPortSyncPulseTime_;
  CSoftwareComponentPort<SOutputImuData, static_cast<uint8_t>(EPortIds::eImuOutput)> oPortImuOutput_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedMagnetometerData)> oPortCompensatedMagnetometerData_;
  CSoftwareComponentPort<SMagneticHeading, static_cast<uint8_t>(EPortIds::eMagneticHeading)> oPortMagneticHeading_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedMagnetometerDataInVehicleFrame)> oPortCompensatedMagnetometerDataInVehicleFrame_;
  CSoftwareComponentPort<CImuDataAfterMonitor, static_cast<uint8_t>(EPortIds::eImuDataAfterMonitor)> oPortImuDataAfterMonitor_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eSafePressureData1)> oPortSafePressureData1_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eSafePressureData2)> oPortSafePressureData2_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eSafePressureData3)> oPortSafePressureData3_;
  CSoftwareComponentPort<CBahrsFilterOutput, static_cast<uint8_t>(EPortIds::eBahrsFilterOutput1)> oPortBahrsFilterOutput1_;
  CSoftwareComponentPort<CBahrsFilterOutput, static_cast<uint8_t>(EPortIds::eBahrsFilterOutput2)> oPortBahrsFilterOutput2_;
  CSoftwareComponentPort<CBahrsFilterOutput, static_cast<uint8_t>(EPortIds::eBahrsFilterOutput3)> oPortBahrsFilterOutput3_;
  CSoftwareComponentPort<SAttitudeData, static_cast<uint8_t>(EPortIds::eVehicleAttitude1)> oPortVehicleAttitude1_;
  CSoftwareComponentPort<SAttitudeData, static_cast<uint8_t>(EPortIds::eVehicleAttitude2)> oPortVehicleAttitude2_;
  CSoftwareComponentPort<SAttitudeData, static_cast<uint8_t>(EPortIds::eVehicleAttitude3)> oPortVehicleAttitude3_;
  CSoftwareComponentPort<SSafeAttitudeData, static_cast<uint8_t>(EPortIds::eSafeVehicleAttitude)> oPortSafeVehicleAttitude_;
  CSoftwareComponentPort<SSafeVerticalChannelData, static_cast<uint8_t>(EPortIds::eSafeVerticalChannelData)> oPortSafeVerticalChannelData_;


};

#endif /* C_RTE_H */
