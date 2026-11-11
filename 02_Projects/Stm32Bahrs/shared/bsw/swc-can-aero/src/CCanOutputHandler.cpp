/// @file CCanOutputHandler.cpp
/// @brief Implementation of the CAN Output Handler software component.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CCanOutputHandler.h"
#include "CRte.h"
#include "CNvmDriver.h"
#include "CGeoConstants.h"
#include "CMathConstants.h"
#include "crc32.h"
#include "UintToBool.h"

using namespace NNavigationUtilities;

CCanOutputHandler& CCanOutputHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CCanOutputHandler soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

const CCanOutputHandler::CFrameHanleArray& CCanOutputHandler::GetFrameHandles()
{
  return GetInstance().apFrameHandles_;
}

CCanOutputHandler::CCanOutputHandler() :
    apFrameHandles_
      {
        SCanFrameHandle( CCanOutputHandler::SendBodyLongitudinalAcceleration, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendBodyLateralAcceleration, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendBodyNormalAcceleration, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendBodyPitchRate, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendBodyRollRate, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendBodyYawRate, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendBodyPitchAngle, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendBodyRollAngle, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendAltitudeRate, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendMagneticHeading, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendStandardAltitude, 80U ),
        SCanFrameHandle( CCanOutputHandler::SendStaticPressure, 10U )
      }
{

}

void CCanOutputHandler::Init()
{
  bool bStatus = false;

  if (false == bIsInitialized_)
  {
    CNvmDriver::SCanConfigurationParameters oCanConfigParameters;

    bStatus = CNvmDriver::ReadBlock<CNvmDriver::CNvmBlockCanConfigurationParameters>(oCanConfigParameters);
    if (true == bStatus)
    {
      Crc32_t uComputedCrc = crc32(reinterpret_cast<void*>(&oCanConfigParameters), sizeof(oCanConfigParameters) - sizeof(Crc32_t));

      if (uComputedCrc == oCanConfigParameters.uCrc_)
      {
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eBodyLongitudinalAcceleration)].uFrameFrequency_  = oCanConfigParameters.uUpdateRateFrame300_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eBodyLateralAcceleration)].uFrameFrequency_       = oCanConfigParameters.uUpdateRateFrame301_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eBodyNormalAcceleration)].uFrameFrequency_        = oCanConfigParameters.uUpdateRateFrame302_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eBodyPitchRate)].uFrameFrequency_                 = oCanConfigParameters.uUpdateRateFrame303_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eBodyRollRate)].uFrameFrequency_                  = oCanConfigParameters.uUpdateRateFrame304_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eBodyYawRate)].uFrameFrequency_                   = oCanConfigParameters.uUpdateRateFrame305_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eBodyPitchAngle)].uFrameFrequency_                = oCanConfigParameters.uUpdateRateFrame311_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eBodyRollAngle)].uFrameFrequency_                 = oCanConfigParameters.uUpdateRateFrame312_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eAltitudeRate)].uFrameFrequency_                  = oCanConfigParameters.uUpdateRateFrame314_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eMagneticHeading)].uFrameFrequency_               = oCanConfigParameters.uUpdateRateFrame1069_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eStandardAltitude)].uFrameFrequency_              = oCanConfigParameters.uUpdateRateFrame322_;
        apFrameHandles_[static_cast<uint8_t>(ECanIds::eStaticPressure)].uFrameFrequency_                = oCanConfigParameters.uUpdateRateFrame326_;
      }
    }

    // Parameter advertisements:
    if (true == bStatus)
    {
      CCanHandler& orInstance = CCanHandler::GetInstance();

      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_LONGITUDINAL_ACCELERATION, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_LATERAL_ACCELERATION, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_NORMAL_ACCELERATION, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_PITCH_RATE, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_ROLL_RATE, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_YAW_RATE, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_PITCH_ANGLE, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_ROLL_ANGLE, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_ALTITUDE_RATE, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_MAGNETIC_HEADING, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_STANDARD_ALTITUDE, false));
      assert(0 == canasParamAdvertise(&orInstance.oCanasInstance, CANAS_NOD_DEF_STATIC_PRESSURE, false));
    }

    // Can output is initialized
    if (true == bStatus)
    {
      bIsInitialized_ = true;
    }
  }
}

bool CCanOutputHandler::IsInitialized()
{
  return bIsInitialized_;
}

void CCanOutputHandler::SendBodyLongitudinalAcceleration()
{
  SOutputImuData oImuData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortImuOutput_.Read(oImuData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eSpecificForceHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oImuData.fSpecificForceX_ / CGeoConstants::skfGravity;

        if (isSignalInRange(kfTmp, -skfMaxSpecificForceInG_, skfMaxSpecificForceInG_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_LONGITUDINAL_ACCELERATION, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendBodyLateralAcceleration()
{
  SOutputImuData oImuData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortImuOutput_.Read(oImuData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eSpecificForceHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oImuData.fSpecificForceY_ / CGeoConstants::skfGravity;

        if (isSignalInRange(kfTmp, -skfMaxSpecificForceInG_, skfMaxSpecificForceInG_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_LATERAL_ACCELERATION, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendBodyNormalAcceleration()
{
  SOutputImuData oImuData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortImuOutput_.Read(oImuData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eSpecificForceHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oImuData.fSpecificForceZ_ / CGeoConstants::skfGravity;

        if (isSignalInRange(kfTmp, -skfMaxSpecificForceInG_, skfMaxSpecificForceInG_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_NORMAL_ACCELERATION, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendBodyPitchRate()
{
  SOutputImuData oImuData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortImuOutput_.Read(oImuData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eAngularRateHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oImuData.fAngularRateY_ * (180.0F / CMathConstants::skfPi_);

        if (isSignalInRange(kfTmp, -skfMaxAngularRateInDegreesPerSecond_, skfMaxAngularRateInDegreesPerSecond_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_PITCH_RATE, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendBodyRollRate()
{
  SOutputImuData oImuData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortImuOutput_.Read(oImuData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eAngularRateHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oImuData.fAngularRateX_ * (180.0F / CMathConstants::skfPi_);

        if (isSignalInRange(kfTmp, -skfMaxAngularRateInDegreesPerSecond_, skfMaxAngularRateInDegreesPerSecond_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_ROLL_RATE, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendBodyYawRate()
{
  SOutputImuData oImuData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortImuOutput_.Read(oImuData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eAngularRateHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oImuData.fAngularRateZ_ * (180.0F / CMathConstants::skfPi_);

        if (isSignalInRange(kfTmp, -skfMaxAngularRateInDegreesPerSecond_, skfMaxAngularRateInDegreesPerSecond_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_YAW_RATE, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendBodyPitchAngle()
{
  SSafeAttitudeData oVehicleAttitude;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortSafeVehicleAttitude_.Read(oVehicleAttitude);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oVehicleAttitude.eHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oVehicleAttitude.fPitch_ * (180.0F / CMathConstants::skfPi_);

        if (isSignalInRange(kfTmp, -skfMaxPitchInDegrees_, skfMaxPitchInDegrees_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_PITCH_ANGLE, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendBodyRollAngle()
{
  SSafeAttitudeData oVehicleAttitude;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortSafeVehicleAttitude_.Read(oVehicleAttitude);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oVehicleAttitude.eHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oVehicleAttitude.fRoll_ * (180.0F / CMathConstants::skfPi_);

        if (isSignalInRange(kfTmp, -skfMaxRollInDegrees_, skfMaxRollInDegrees_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_BODY_ROLL_ANGLE, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendAltitudeRate()
{
  SSafeVerticalChannelData oVerticalData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortSafeVerticalChannelData_.Read(oVerticalData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oVerticalData.eHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = -oVerticalData.fVelocityDown_;

        if (isSignalInRange(kfTmp, -skfMaxVelocityDown_, skfMaxVelocityDown_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_ALTITUDE_RATE, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendMagneticHeading()
{
  SSafeMagneticHeadingData oMagneticHeadingData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortSafeMagneticHeading_.Read(oMagneticHeadingData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oMagneticHeadingData.eHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oMagneticHeadingData.fMagneticHeading_ * (180.0F / CMathConstants::skfPi_);

        if (isSignalInRange(kfTmp, -skfMaxHeadingInDegrees_, skfMaxHeadingInDegrees_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_MAGNETIC_HEADING, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendStandardAltitude()
{
  SSafeVerticalChannelData oVerticalChannelData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  bool bReadStatus = CRte::GetInstance().oPortSafeVerticalChannelData_.Read(oVerticalChannelData);
  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (CSerialProtocol::ESignalHealthInfo::eUnavailable != oVerticalChannelData.eHealth_))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        if (isSignalInRange(oVerticalChannelData.fHeight_, skfMinAltitude_, skfMaxAltitude_))
        {
          oMsgd.container.FLOAT = oVerticalChannelData.fHeight_;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_STANDARD_ALTITUDE, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

void CCanOutputHandler::SendStaticPressure()
{
  SBarometerMeasurement oPressureData;
  CanasMessageData oMsgd;
  uint8_t uServiceCode;

  // Read the first safe pressure data source
  bool bReadStatus = CRte::GetInstance().oPortSafePressureData1_.Read(oPressureData);

  if ((false == bReadStatus) || (false == UintToBool(oPressureData.uValid_)))
  {
    // The first source is not available, read the second
    bReadStatus = CRte::GetInstance().oPortSafePressureData2_.Read(oPressureData);

    if ((false == bReadStatus) || (false == UintToBool(oPressureData.uValid_)))
    {
      // The second is not available, read the third
      bReadStatus = CRte::GetInstance().oPortSafePressureData3_.Read(oPressureData);
    }
  }

  CCanHandler& orInstance = CCanHandler::GetInstance();

  if ((true == bReadStatus) && (true == UintToBool(oPressureData.uValid_)))
  {
    if (osOK == osMutexAcquire(orInstance.oCanasPlatform.pMutex, CCanHandler::skuMutexTout))
    {
        oMsgd.type = CANAS_DATATYPE_FLOAT;

        const float kfTmp = oPressureData.fPressure_ / 100.0F;

        if (isSignalInRange(kfTmp, skfMinPressureInHPa_, skfMaxPressureInHPa_))
        {
          oMsgd.container.FLOAT = kfTmp;
          uServiceCode = 0;
          canasParamPublish(&orInstance.oCanasInstance, CANAS_NOD_DEF_STATIC_PRESSURE, &oMsgd, uServiceCode);
        }

        osMutexRelease(orInstance.oCanasPlatform.pMutex);
    }
  }
}

uint32_t CCanOutputHandler::HerzToMs(uint32_t uFrequency)
{
  uint32_t period = 0;

  if (uFrequency > 0) {
    period = (1000 / uFrequency);
  }
  return period;
}

bool CCanOutputHandler::isSignalInRange(float fSignal, float fSignalMin, float fSignalMax)
{
  bool bResult{false};

  if (std::isfinite(fSignalMin) && std::isfinite(fSignalMax) && std::isfinite(fSignal))
  {
    if ((fSignal <= fSignalMax) && (fSignal >= fSignalMin))
    {
      bResult = true;
    }
  }

  return bResult;
}
