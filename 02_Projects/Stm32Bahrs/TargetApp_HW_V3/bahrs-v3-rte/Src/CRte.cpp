/**
 * @file CRte.cpp
 * @brief Implementation of the runtime environment class.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date 29.05.2026 at 10:43:47
 */

#include "CRte.h"

CRte& CRte::GetInstance()
{
  static CRte soRteInstance;
  return soRteInstance;
}

CRte::CRte()
{
}

void CRte::Init()
{
  oPortImuInput1_.Init();
  oPortImuInput2_.Init();
  oPortImuInput3_.Init();
  oPortPressureInput1_.Init();
  oPortPressureInput2_.Init();
  oPortPressureInput3_.Init();
  oPortCompensatedPressureInput1_.Init();
  oPortCompensatedPressureInput2_.Init();
  oPortCompensatedPressureInput3_.Init();
  oPortMagnetometerInput1_.Init();
  oPortMagnetometerInput2_.Init();
  oPortMagnetometerInput3_.Init();
  oPortSyncPulseTime_.Init();
  oPortImuOutput_.Init();
  oPortCompensatedMagnetometerData_.Init();
  oPortMagneticHeading_.Init();
  oPortCompensatedMagnetometerDataInVehicleFrame_.Init();
  oPortImuDataAfterMonitor_.Init();
  oPortSafePressureData1_.Init();
  oPortSafePressureData2_.Init();
  oPortSafePressureData3_.Init();
  oPortBahrsFilterOutput1_.Init();
  oPortBahrsFilterOutput2_.Init();
  oPortBahrsFilterOutput3_.Init();
  oPortVehicleAttitude1_.Init();
  oPortVehicleAttitude2_.Init();
  oPortVehicleAttitude3_.Init();
  oPortSafeVehicleAttitude_.Init();
  oPortSafeVerticalChannelData_.Init();

#ifdef SEND_DEBUG_OUTPUT
  oPortImuInput1_.SetRecordForDebug(true);
  oPortImuInput2_.SetRecordForDebug(true);
  oPortImuInput3_.SetRecordForDebug(true);
  oPortPressureInput1_.SetRecordForDebug(true);
  oPortPressureInput2_.SetRecordForDebug(true);
  oPortPressureInput3_.SetRecordForDebug(true);
  oPortMagnetometerInput1_.SetRecordForDebug(true);
  oPortMagnetometerInput2_.SetRecordForDebug(true);
  oPortMagnetometerInput3_.SetRecordForDebug(true);
  oPortSyncPulseTime_.SetRecordForDebug(true);
  oPortSafeVehicleAttitude_.SetRecordForDebug(true);
  oPortSafeVerticalChannelData_.SetRecordForDebug(true);
#endif /* SEND_DEBUG_OUTPUT */

}

