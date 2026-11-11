/**
 * @file WriteToRtePort.cpp
 * @brief Fill RTE ports with debug output data.
 * 
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date 23.10.2026 at 14:50:40
 */

#include "WriteToRtePort.h"

void WriteToRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes)
{
  switch (ePortId)
  {
    
    case CRte::EPortIds::eImuInput1:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortImuInput1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInput2:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortImuInput2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInput3:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortImuInput3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::ePressureInput1:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortPressureInput1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::ePressureInput2:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortPressureInput2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::ePressureInput3:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortPressureInput3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedPressureInput1:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedPressureInput1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedPressureInput2:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedPressureInput2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedPressureInput3:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedPressureInput3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagnetometerInput1:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortMagnetometerInput1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagnetometerInput2:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortMagnetometerInput2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagnetometerInput3:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortMagnetometerInput3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eSyncPulseTime:
      {
        STimeOfSyncPulse oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortSyncPulseTime_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerData1:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedMagnetometerData1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagneticHeading1:
      {
        SMagneticHeading oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortMagneticHeading1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame1:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eSafePressureData1:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortSafePressureData1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eSafePressureData2:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortSafePressureData2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eSafePressureData3:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortSafePressureData3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eVehicleAttitude1:
      {
        SAttitudeData oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortVehicleAttitude1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eVehicleAttitude2:
      {
        SAttitudeData oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortVehicleAttitude2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eVehicleAttitude3:
      {
        SAttitudeData oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortVehicleAttitude3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInputFiltered1:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortImuInputFiltered1_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInputFiltered2:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortImuInputFiltered2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInputFiltered3:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortImuInputFiltered3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerData2:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedMagnetometerData2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerData3:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedMagnetometerData3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagneticHeading2:
      {
        SMagneticHeading oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortMagneticHeading2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagneticHeading3:
      {
        SMagneticHeading oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortMagneticHeading3_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame2:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame2_.Write(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame3:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(korBytes))
        {
          CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame3_.Write(oPortData);
        }
      }
      break;

  default:
    WriteToUserDefinedRtePort(ePortId, korBytes);
    break;
  }
}
