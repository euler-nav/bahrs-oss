/**
 * @file CRte.h
 * @brief Declaration of the runtime environment class.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date 23.10.2026 at 14:50:40
 */

#ifndef C_RTE_H
#define C_RTE_H

#include "CSoftwareComponentPort.h"
#include "RteTypesGenerated.h"
#include "RteTypesUserDefined.h"
#include "General/HelperSoftwareComponentMacros.h"

#include "COutputTransformer.h"
#include "CPressureCompensator.h"
#include "CImuPreFilterSwc.h"
#include "CMagnetometerCompensator.h"
#include "CMagneticHeadingFilterSwc.h"
#include "CImuMonitorSwc.h"
#include "CBaroMonitorSwc.h"
#include "CBahrsFilterSwc.h"
#include "CAttitudeMonitorSwc.h"
#include "CVerticalChannelMonitorSwc.h"
#include "CMagneticHeadingMonitorSwc.h"
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
#include "CNvmDriver.h"
#include "CCanStackAliases.h"
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

  friend COutputTransformer;
  friend CPressureCompensator;
  friend CImuPreFilterSwc;
  friend CMagnetometerCompensator;
  friend CMagneticHeadingFilterSwc;
  friend CImuMonitorSwc;
  friend CBaroMonitorSwc;
  friend CBahrsFilterSwc;
  friend CAttitudeMonitorSwc;
  friend CVerticalChannelMonitorSwc;
  friend CMagneticHeadingMonitorSwc;
#ifndef _MSC_VER
  friend CBmi270Driver;
  friend CAsm330lhhDriver;
  friend CRs232OutputHandler;
  friend CSpiHandler;
  friend CI2CHandler;
  friend CBmp384Driver;
  friend CLps22hhDriver;
  friend CScha63TDriver;
  friend CIcp20100Driver;
  friend CMmc5983Driver;
  friend CLis3mdlDriver;
  friend CBmm350Driver;
  friend CNvmDriver;
  friend CCanHandler;
  friend CCanOutputHandler;
  friend CSyncPulseHandler;
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
    eCompensatedMagnetometerData1,
    eMagneticHeading1,
    eCompensatedMagnetometerDataInVehicleFrame1,
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
    eSafeVerticalChannelData,
    eImuInputFiltered1,
    eImuInputFiltered2,
    eImuInputFiltered3,
    eCompensatedMagnetometerData2,
    eCompensatedMagnetometerData3,
    eMagneticHeading2,
    eMagneticHeading3,
    eCompensatedMagnetometerDataInVehicleFrame2,
    eCompensatedMagnetometerDataInVehicleFrame3,
    eSafeMagneticHeading
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

  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInput1), 1000U> oPortImuInput1_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInput2), 1000U> oPortImuInput2_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInput3), 1000U> oPortImuInput3_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePressureInput1), 1000U> oPortPressureInput1_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePressureInput2), 1000U> oPortPressureInput2_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::ePressureInput3), 1000U> oPortPressureInput3_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedPressureInput1), 1000U> oPortCompensatedPressureInput1_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedPressureInput2), 1000U> oPortCompensatedPressureInput2_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedPressureInput3), 1000U> oPortCompensatedPressureInput3_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eMagnetometerInput1), 1000U> oPortMagnetometerInput1_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eMagnetometerInput2), 1000U> oPortMagnetometerInput2_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eMagnetometerInput3), 1000U> oPortMagnetometerInput3_;
  CSoftwareComponentPort<STimeOfSyncPulse, static_cast<uint8_t>(EPortIds::eSyncPulseTime), 0U> oPortSyncPulseTime_;
  CSoftwareComponentPort<SOutputImuData, static_cast<uint8_t>(EPortIds::eImuOutput), 1000U> oPortImuOutput_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedMagnetometerData1), 1000U> oPortCompensatedMagnetometerData1_;
  CSoftwareComponentPort<SMagneticHeading, static_cast<uint8_t>(EPortIds::eMagneticHeading1), 1000U> oPortMagneticHeading1_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedMagnetometerDataInVehicleFrame1), 1000U> oPortCompensatedMagnetometerDataInVehicleFrame1_;
  CSoftwareComponentPort<CImuDataAfterMonitor, static_cast<uint8_t>(EPortIds::eImuDataAfterMonitor), 1000U> oPortImuDataAfterMonitor_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eSafePressureData1), 1000U> oPortSafePressureData1_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eSafePressureData2), 1000U> oPortSafePressureData2_;
  CSoftwareComponentPort<SBarometerMeasurement, static_cast<uint8_t>(EPortIds::eSafePressureData3), 1000U> oPortSafePressureData3_;
  CSoftwareComponentPort<CBahrsFilterOutput, static_cast<uint8_t>(EPortIds::eBahrsFilterOutput1), 1000U> oPortBahrsFilterOutput1_;
  CSoftwareComponentPort<CBahrsFilterOutput, static_cast<uint8_t>(EPortIds::eBahrsFilterOutput2), 1000U> oPortBahrsFilterOutput2_;
  CSoftwareComponentPort<CBahrsFilterOutput, static_cast<uint8_t>(EPortIds::eBahrsFilterOutput3), 1000U> oPortBahrsFilterOutput3_;
  CSoftwareComponentPort<SAttitudeData, static_cast<uint8_t>(EPortIds::eVehicleAttitude1), 1000U> oPortVehicleAttitude1_;
  CSoftwareComponentPort<SAttitudeData, static_cast<uint8_t>(EPortIds::eVehicleAttitude2), 1000U> oPortVehicleAttitude2_;
  CSoftwareComponentPort<SAttitudeData, static_cast<uint8_t>(EPortIds::eVehicleAttitude3), 1000U> oPortVehicleAttitude3_;
  CSoftwareComponentPort<SSafeAttitudeData, static_cast<uint8_t>(EPortIds::eSafeVehicleAttitude), 1000U> oPortSafeVehicleAttitude_;
  CSoftwareComponentPort<SSafeVerticalChannelData, static_cast<uint8_t>(EPortIds::eSafeVerticalChannelData), 1000U> oPortSafeVerticalChannelData_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInputFiltered1), 1000U> oPortImuInputFiltered1_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInputFiltered2), 1000U> oPortImuInputFiltered2_;
  CSoftwareComponentPort<SImuMeasurement, static_cast<uint8_t>(EPortIds::eImuInputFiltered3), 1000U> oPortImuInputFiltered3_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedMagnetometerData2), 1000U> oPortCompensatedMagnetometerData2_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedMagnetometerData3), 1000U> oPortCompensatedMagnetometerData3_;
  CSoftwareComponentPort<SMagneticHeading, static_cast<uint8_t>(EPortIds::eMagneticHeading2), 1000U> oPortMagneticHeading2_;
  CSoftwareComponentPort<SMagneticHeading, static_cast<uint8_t>(EPortIds::eMagneticHeading3), 1000U> oPortMagneticHeading3_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedMagnetometerDataInVehicleFrame2), 1000U> oPortCompensatedMagnetometerDataInVehicleFrame2_;
  CSoftwareComponentPort<SMagneticMeasurement, static_cast<uint8_t>(EPortIds::eCompensatedMagnetometerDataInVehicleFrame3), 1000U> oPortCompensatedMagnetometerDataInVehicleFrame3_;
  CSoftwareComponentPort<SSafeMagneticHeadingData, static_cast<uint8_t>(EPortIds::eSafeMagneticHeading), 1000U> oPortSafeMagneticHeading_;


};

#endif /* C_RTE_H */
