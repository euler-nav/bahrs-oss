/// @file WriteUserDefinedDataPorts.cpp
/// @brief Implementation of functions for exporting user-defined data ports to MAT file.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "WriteUserDefinedDataPorts.h"
#include <iostream>

using namespace std;

static vector<SOutputImuData> soPortDataOutputImuDataContainer;
static vector<SSafeAttitudeData> soPortDataSafeAttitudeDataContainer;
static vector<SSafeVerticalChannelData> soPortDataSafeVerticalChannelDataContainer;
static vector<SSafeMagneticHeadingData> soPortDataSafeMagneticHeadingDataContainer;
static vector<CBahrsFilterOutput> soPortDataBahrsFilterOutput1Container;
static vector<CBahrsFilterOutput> soPortDataBahrsFilterOutput2Container;
static vector<CBahrsFilterOutput> soPortDataBahrsFilterOutput3Container;
static vector<CImuDataAfterMonitor> soPortDataImuDataAfterMonitorContainer;

static void writePortOutputImuDataToMat(TinyMATWriterFile* opMatWriter, std::vector<SOutputImuData>& orPortDataContainer);
static void writePortSafeAttitudeDataToMat(TinyMATWriterFile* opMatWriter, std::vector<SSafeAttitudeData>& orPortDataContainer);
static void writePortSafeVerticalChannelDataToMat(TinyMATWriterFile* opMatWriter, std::vector<SSafeVerticalChannelData>& orPortDataContainer);
static void writePortSafeMagneticHeadingDataToMat(TinyMATWriterFile* opMatWriter, std::vector<SSafeMagneticHeadingData>& orPortDataContainer);
static void writePortBahrsFilterOutputToMat(TinyMATWriterFile* opMatWriter, std::vector<CBahrsFilterOutput>& orPortDataContainer, const char* kpStructName);
static void writePortImuDataAfterMonitorToMat(TinyMATWriterFile* opMatWriter, std::vector<CImuDataAfterMonitor>& orPortDataContainer);

void BufferDataFromUserDefinedPorts(CRte::EPortIds ePortId, const vector<uint8_t>& korBytes)
{
  switch (ePortId)
  {
  case CRte::EPortIds::eImuOutput:
  {
    SOutputImuData oPortData;

    if (oPortData.FromByteVector(korBytes))
    {
      soPortDataOutputImuDataContainer.push_back(oPortData);
    }
  }
  break;
  case CRte::EPortIds::eSafeVehicleAttitude:
  {
    SSafeAttitudeData oPortData;

    if (oPortData.FromByteVector(korBytes))
    {
      soPortDataSafeAttitudeDataContainer.push_back(oPortData);
    }
  }
  break;
  case CRte::EPortIds::eSafeVerticalChannelData:
  {
    SSafeVerticalChannelData oPortData;

    if (oPortData.FromByteVector(korBytes))
    {
      soPortDataSafeVerticalChannelDataContainer.push_back(oPortData);
    }
  }
  break;
  case CRte::EPortIds::eSafeMagneticHeading:
  {
    SSafeMagneticHeadingData oPortData;

    if (oPortData.FromByteVector(korBytes))
    {
      soPortDataSafeMagneticHeadingDataContainer.push_back(oPortData);
    }
  }
  break;
  case CRte::EPortIds::eBahrsFilterOutput1:
  {
    CBahrsFilterOutput oPortData;

    if (oPortData.FromByteVector(korBytes))
    {
      soPortDataBahrsFilterOutput1Container.push_back(oPortData);
    }
  }
  break;
  case CRte::EPortIds::eBahrsFilterOutput2:
  {
    CBahrsFilterOutput oPortData;

    if (oPortData.FromByteVector(korBytes))
    {
      soPortDataBahrsFilterOutput2Container.push_back(oPortData);
    }
  }
  break;
  case CRte::EPortIds::eBahrsFilterOutput3:
  {
    CBahrsFilterOutput oPortData;

    if (oPortData.FromByteVector(korBytes))
    {
      soPortDataBahrsFilterOutput3Container.push_back(oPortData);
    }
  }
  break;
  case CRte::EPortIds::eImuDataAfterMonitor:
  {
    CImuDataAfterMonitor oPortData;

    if (oPortData.FromByteVector(korBytes))
    {
      soPortDataImuDataAfterMonitorContainer.push_back(oPortData);
    }
  }
  break;
  default:
    break;
  }
}

void WriteUserDefinedPortsToMat(TinyMATWriterFile* opMatWriter)
{
  if (nullptr != opMatWriter)
  {
    writePortOutputImuDataToMat(opMatWriter, soPortDataOutputImuDataContainer);
    writePortSafeAttitudeDataToMat(opMatWriter, soPortDataSafeAttitudeDataContainer);
    writePortSafeVerticalChannelDataToMat(opMatWriter, soPortDataSafeVerticalChannelDataContainer);
    writePortSafeMagneticHeadingDataToMat(opMatWriter, soPortDataSafeMagneticHeadingDataContainer);
    writePortBahrsFilterOutputToMat(opMatWriter, soPortDataBahrsFilterOutput1Container, "PortBahrsFilterOutput1");
    writePortBahrsFilterOutputToMat(opMatWriter, soPortDataBahrsFilterOutput2Container, "PortBahrsFilterOutput2");
    writePortBahrsFilterOutputToMat(opMatWriter, soPortDataBahrsFilterOutput3Container, "PortBahrsFilterOutput3");
    writePortImuDataAfterMonitorToMat(opMatWriter, soPortDataImuDataAfterMonitorContainer);
  }
}

void writePortOutputImuDataToMat(TinyMATWriterFile* opMatWriter, std::vector<SOutputImuData>& orPortDataContainer)
{
  float* fpSpecificForceX{ nullptr };
  float* fpSpecificForceY{ nullptr };
  float* fpSpecificForceZ{ nullptr };
  float* fpAngularRateX{ nullptr };
  float* fpAngularRateY{ nullptr };
  float* fpAngularRateZ{ nullptr };
  uint64_t* upTimestampUs{ nullptr };
  uint8_t* upSpecificForceHealth{ nullptr };
  uint8_t* upAngularRateHealth{ nullptr };

  try
  {
    fpSpecificForceX = new float[orPortDataContainer.size()];
    fpSpecificForceY = new float[orPortDataContainer.size()];
    fpSpecificForceZ = new float[orPortDataContainer.size()];
    fpAngularRateX = new float[orPortDataContainer.size()];
    fpAngularRateY = new float[orPortDataContainer.size()];
    fpAngularRateZ = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upSpecificForceHealth = new uint8_t[orPortDataContainer.size()];
    upAngularRateHealth = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpSpecificForceX[uIndex] = orPortDataContainer[uIndex].fSpecificForceX_;
      fpSpecificForceY[uIndex] = orPortDataContainer[uIndex].fSpecificForceY_;
      fpSpecificForceZ[uIndex] = orPortDataContainer[uIndex].fSpecificForceZ_;
      fpAngularRateX[uIndex] = orPortDataContainer[uIndex].fAngularRateX_;
      fpAngularRateY[uIndex] = orPortDataContainer[uIndex].fAngularRateY_;
      fpAngularRateZ[uIndex] = orPortDataContainer[uIndex].fAngularRateZ_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upSpecificForceHealth[uIndex] = static_cast<uint8_t>(orPortDataContainer[uIndex].eSpecificForceHealth_);
      upAngularRateHealth[uIndex] = static_cast<uint8_t>(orPortDataContainer[uIndex].eAngularRateHealth_);
    }

    TinyMATWriter_startStruct(opMatWriter, "PortOutputImuData");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceX_", fpSpecificForceX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceY_", fpSpecificForceY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceZ_", fpSpecificForceZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateX_", fpAngularRateX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateY_", fpAngularRateY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateZ_", fpAngularRateZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eSpecificForceHealth_", upSpecificForceHealth, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eAngularRateHealth_", upAngularRateHealth, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (const std::bad_alloc&)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortOutputImuData..." << endl;
  }

  delete[] fpSpecificForceX;
  delete[] fpSpecificForceY;
  delete[] fpSpecificForceZ;
  delete[] fpAngularRateX;
  delete[] fpAngularRateY;
  delete[] fpAngularRateZ;
  delete[] upTimestampUs;
  delete[] upSpecificForceHealth;
  delete[] upAngularRateHealth;
}

void writePortSafeAttitudeDataToMat(TinyMATWriterFile* opMatWriter, std::vector<SSafeAttitudeData>& orPortDataContainer)
{
  float* fpRoll{ nullptr };
  float* fpPitch{ nullptr };
  float* fpAttitudeStd1{ nullptr };
  float* fpAttitudeStd2{ nullptr };
  uint64_t* upTimestampUs{ nullptr };
  uint8_t* upHealth{ nullptr };

  try
  {
    fpRoll = new float[orPortDataContainer.size()];
    fpPitch = new float[orPortDataContainer.size()];
    fpAttitudeStd1 = new float[orPortDataContainer.size()];
    fpAttitudeStd2 = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upHealth = new uint8_t[orPortDataContainer.size()];

    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpRoll[uIndex] = orPortDataContainer[uIndex].fRoll_;
      fpPitch[uIndex] = orPortDataContainer[uIndex].fPitch_;
      fpAttitudeStd1[uIndex] = orPortDataContainer[uIndex].fAttitudeStd1_;
      fpAttitudeStd2[uIndex] = orPortDataContainer[uIndex].fAttitudeStd2_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upHealth[uIndex] = static_cast<uint8_t>(orPortDataContainer[uIndex].eHealth_);
    }

    TinyMATWriter_startStruct(opMatWriter, "PortSafeAttitudeData");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fRoll_", fpRoll, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPitch_", fpPitch, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAttitudeStd1_", fpAttitudeStd1, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAttitudeStd2_", fpAttitudeStd2, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eHealth_", upHealth, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (const std::bad_alloc&)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortSafeAttitudeData..." << endl;
  }

  delete[] fpRoll;
  delete[] fpPitch;
  delete[] fpAttitudeStd1;
  delete[] fpAttitudeStd2;
  delete[] upTimestampUs;
  delete[] upHealth;
}

void writePortSafeVerticalChannelDataToMat(TinyMATWriterFile* opMatWriter, std::vector<SSafeVerticalChannelData>& orPortDataContainer)
{
  float* fpHeight{ nullptr };
  float* fpVerticalVelocity{ nullptr };
  uint64_t* upTimestampUs{ nullptr };
  uint8_t* upHealth{ nullptr };

  try
  {
    fpHeight = new float[orPortDataContainer.size()];
    fpVerticalVelocity = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upHealth = new uint8_t[orPortDataContainer.size()];

    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpHeight[uIndex] = orPortDataContainer[uIndex].fHeight_;
      fpVerticalVelocity[uIndex] = orPortDataContainer[uIndex].fVelocityDown_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upHealth[uIndex] = static_cast<uint8_t>(orPortDataContainer[uIndex].eHealth_);
    }

    TinyMATWriter_startStruct(opMatWriter, "PortSafeVerticalChannelData");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fHeight_", fpHeight, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVelocityDown_", fpVerticalVelocity, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eHealth_", upHealth, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (const std::bad_alloc&)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortSafeVerticalChannelData..." << endl;
  }

  delete[] fpHeight;
  delete[] fpVerticalVelocity;
  delete[] upTimestampUs;
  delete[] upHealth;
}

void writePortSafeMagneticHeadingDataToMat(TinyMATWriterFile* opMatWriter, std::vector<SSafeMagneticHeadingData>& orPortDataContainer)
{
  float* fpMagneticHeading{ nullptr };
  uint64_t* upTimestampUs{ nullptr };
  uint8_t* upHealth{ nullptr };

  try
  {
    fpMagneticHeading = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upHealth = new uint8_t[orPortDataContainer.size()];

    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpMagneticHeading[uIndex] = orPortDataContainer[uIndex].fMagneticHeading_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upHealth[uIndex] = static_cast<uint8_t>(orPortDataContainer[uIndex].eHealth_);
    }

    TinyMATWriter_startStruct(opMatWriter, "PortSafeMagneticHeading");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fMagneticHeading_", fpMagneticHeading, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eHealth_", upHealth, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (const std::bad_alloc&)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortSafeMagneticHeading..." << endl;
  }

  delete[] fpMagneticHeading;
  delete[] upTimestampUs;
  delete[] upHealth;
}

void writePortBahrsFilterOutputToMat(TinyMATWriterFile* opMatWriter, std::vector<CBahrsFilterOutput>& orPortDataContainer, const char* kpStructName)
{
  float* fpHeight{ nullptr };
  float* fpVelocityDown{ nullptr };
  float* fpQuatW{ nullptr };
  float* fpQuatX{ nullptr };
  float* fpQuatY{ nullptr };
  float* fpQuatZ{ nullptr };
  float* fpAccBiasX{ nullptr };
  float* fpAccBiasY{ nullptr };
  float* fpAccBiasZ{ nullptr };
  float* fpGyrBiasX{ nullptr };
  float* fpGyrBiasY{ nullptr };
  float* fpGyrBiasZ{ nullptr };
  float* fpAccScaleX{ nullptr };
  float* fpAccScaleY{ nullptr };
  float* fpAccScaleZ{ nullptr };
  float* fpGyrScaleX{ nullptr };
  float* fpGyrScaleY{ nullptr };
  float* fpGyrScaleZ{ nullptr };
  float* fpAccNedX{ nullptr };
  float* fpAccNedY{ nullptr };
  float* fpAccNedZ{ nullptr };
  float* fpHeightStd{ nullptr };
  float* fpVelocityDownStd{ nullptr };
  float* fpAttStdX{ nullptr };
  float* fpAttStdY{ nullptr };
  float* fpAttStdZ{ nullptr };
  float* fpAccBiasStdX{ nullptr };
  float* fpAccBiasStdY{ nullptr };
  float* fpAccBiasStdZ{ nullptr };
  float* fpGyrBiasStdX{ nullptr };
  float* fpGyrBiasStdY{ nullptr };
  float* fpGyrBiasStdZ{ nullptr };
  float* fpAccScaleStdX{ nullptr };
  float* fpAccScaleStdY{ nullptr };
  float* fpAccScaleStdZ{ nullptr };
  float* fpGyrScaleStdX{ nullptr };
  float* fpGyrScaleStdY{ nullptr };
  float* fpGyrScaleStdZ{ nullptr };
  float* fpAccNedStdX{ nullptr };
  float* fpAccNedStdY{ nullptr };
  float* fpAccNedStdZ{ nullptr };
  uint64_t* upTimestampUs{ nullptr };
  uint8_t* upFilterMode{ nullptr };
  uint8_t* upVerticalChannelDiverged{ nullptr };

  try
  {
    const size_t uSize = orPortDataContainer.size();
    fpHeight = new float[uSize];
    fpVelocityDown = new float[uSize];
    fpQuatW = new float[uSize];
    fpQuatX = new float[uSize];
    fpQuatY = new float[uSize];
    fpQuatZ = new float[uSize];
    fpAccBiasX = new float[uSize];
    fpAccBiasY = new float[uSize];
    fpAccBiasZ = new float[uSize];
    fpGyrBiasX = new float[uSize];
    fpGyrBiasY = new float[uSize];
    fpGyrBiasZ = new float[uSize];
    fpAccScaleX = new float[uSize];
    fpAccScaleY = new float[uSize];
    fpAccScaleZ = new float[uSize];
    fpGyrScaleX = new float[uSize];
    fpGyrScaleY = new float[uSize];
    fpGyrScaleZ = new float[uSize];
    fpAccNedX = new float[uSize];
    fpAccNedY = new float[uSize];
    fpAccNedZ = new float[uSize];
    fpHeightStd = new float[uSize];
    fpVelocityDownStd = new float[uSize];
    fpAttStdX = new float[uSize];
    fpAttStdY = new float[uSize];
    fpAttStdZ = new float[uSize];
    fpAccBiasStdX = new float[uSize];
    fpAccBiasStdY = new float[uSize];
    fpAccBiasStdZ = new float[uSize];
    fpGyrBiasStdX = new float[uSize];
    fpGyrBiasStdY = new float[uSize];
    fpGyrBiasStdZ = new float[uSize];
    fpAccScaleStdX = new float[uSize];
    fpAccScaleStdY = new float[uSize];
    fpAccScaleStdZ = new float[uSize];
    fpGyrScaleStdX = new float[uSize];
    fpGyrScaleStdY = new float[uSize];
    fpGyrScaleStdZ = new float[uSize];
    fpAccNedStdX = new float[uSize];
    fpAccNedStdY = new float[uSize];
    fpAccNedStdZ = new float[uSize];
    upTimestampUs = new uint64_t[uSize];
    upFilterMode = new uint8_t[uSize];
    upVerticalChannelDiverged = new uint8_t[uSize];

    for (uint64_t uIndex = 0; uIndex < uSize; uIndex++)
    {
      const auto& orData = orPortDataContainer[uIndex];
      fpHeight[uIndex] = orData.oState_.fHeight_;
      fpVelocityDown[uIndex] = orData.oState_.fVelocityDown_;
      fpQuatW[uIndex] = orData.oState_.oQuaternionBodyToNed_.W();
      fpQuatX[uIndex] = orData.oState_.oQuaternionBodyToNed_.X();
      fpQuatY[uIndex] = orData.oState_.oQuaternionBodyToNed_.Y();
      fpQuatZ[uIndex] = orData.oState_.oQuaternionBodyToNed_.Z();
      fpAccBiasX[uIndex] = orData.oState_.oAccelerometerBias_.x();
      fpAccBiasY[uIndex] = orData.oState_.oAccelerometerBias_.y();
      fpAccBiasZ[uIndex] = orData.oState_.oAccelerometerBias_.z();
      fpGyrBiasX[uIndex] = orData.oState_.oGyroscopeBias_.x();
      fpGyrBiasY[uIndex] = orData.oState_.oGyroscopeBias_.y();
      fpGyrBiasZ[uIndex] = orData.oState_.oGyroscopeBias_.z();
      fpAccScaleX[uIndex] = orData.oState_.oAccelerometerScaleFactor_.x();
      fpAccScaleY[uIndex] = orData.oState_.oAccelerometerScaleFactor_.y();
      fpAccScaleZ[uIndex] = orData.oState_.oAccelerometerScaleFactor_.z();
      fpGyrScaleX[uIndex] = orData.oState_.oGyroscopeScaleFactor_.x();
      fpGyrScaleY[uIndex] = orData.oState_.oGyroscopeScaleFactor_.y();
      fpGyrScaleZ[uIndex] = orData.oState_.oGyroscopeScaleFactor_.z();
      fpAccNedX[uIndex] = orData.oState_.oAccelerationNed_.x();
      fpAccNedY[uIndex] = orData.oState_.oAccelerationNed_.y();
      fpAccNedZ[uIndex] = orData.oState_.oAccelerationNed_.z();
      fpHeightStd[uIndex] = orData.oStateStd_.fHeight_;
      fpVelocityDownStd[uIndex] = orData.oStateStd_.fVelocityDown_;
      fpAttStdX[uIndex] = orData.oStateStd_.oAttitude_.x();
      fpAttStdY[uIndex] = orData.oStateStd_.oAttitude_.y();
      fpAttStdZ[uIndex] = orData.oStateStd_.oAttitude_.z();
      fpAccBiasStdX[uIndex] = orData.oStateStd_.oAccelerometerBias_.x();
      fpAccBiasStdY[uIndex] = orData.oStateStd_.oAccelerometerBias_.y();
      fpAccBiasStdZ[uIndex] = orData.oStateStd_.oAccelerometerBias_.z();
      fpGyrBiasStdX[uIndex] = orData.oStateStd_.oGyroscopeBias_.x();
      fpGyrBiasStdY[uIndex] = orData.oStateStd_.oGyroscopeBias_.y();
      fpGyrBiasStdZ[uIndex] = orData.oStateStd_.oGyroscopeBias_.z();
      fpAccScaleStdX[uIndex] = orData.oStateStd_.oAccelerometerScaleFactor_.x();
      fpAccScaleStdY[uIndex] = orData.oStateStd_.oAccelerometerScaleFactor_.y();
      fpAccScaleStdZ[uIndex] = orData.oStateStd_.oAccelerometerScaleFactor_.z();
      fpGyrScaleStdX[uIndex] = orData.oStateStd_.oGyroscopeScaleFactor_.x();
      fpGyrScaleStdY[uIndex] = orData.oStateStd_.oGyroscopeScaleFactor_.y();
      fpGyrScaleStdZ[uIndex] = orData.oStateStd_.oGyroscopeScaleFactor_.z();
      fpAccNedStdX[uIndex] = orData.oStateStd_.oAccelerationNed_.x();
      fpAccNedStdY[uIndex] = orData.oStateStd_.oAccelerationNed_.y();
      fpAccNedStdZ[uIndex] = orData.oStateStd_.oAccelerationNed_.z();
      upTimestampUs[uIndex] = orData.uTimestampUs_;
      upFilterMode[uIndex] = static_cast<uint8_t>(orData.eFilterMode_);
      upVerticalChannelDiverged[uIndex] = orData.bVerticalChannelDiverged_ ? 1U : 0U;
    }

    const auto iSize = static_cast<int32_t>(orPortDataContainer.size());

    auto write3xN = [&](const char* name, const float* fpX, const float* fpY, const float* fpZ)
    {
      std::vector<float> buf(3 * iSize);

      for (int32_t i = 0; i < iSize; ++i)
      {
        buf[i] = fpX[i];
        buf[iSize + i] = fpY[i];
        buf[2 * iSize + i] = fpZ[i];
      }

      TinyMATWriter_writeMatrix2D_rowmajor(opMatWriter, name, buf.data(), iSize, 3);
    };

    auto write4xN = [&](const char* name, const float* fp0, const float* fp1, const float* fp2, const float* fp3)
    {
      std::vector<float> buf(4 * iSize);

      for (int32_t i = 0; i < iSize; ++i)
      {
        buf[i] = fp0[i];
        buf[iSize + i] = fp1[i];
        buf[2 * iSize + i] = fp2[i];
        buf[3 * iSize + i] = fp3[i];
      }
      
      TinyMATWriter_writeMatrix2D_rowmajor(opMatWriter, name, buf.data(), iSize, 4);
    };

    TinyMATWriter_startStruct(opMatWriter, kpStructName);

    TinyMATWriter_startStruct(opMatWriter, "oState_");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fHeight_", fpHeight, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVelocityDown_", fpVelocityDown, iSize);
    write4xN("oQuaternionBodyToNed_", fpQuatW, fpQuatX, fpQuatY, fpQuatZ);
    write3xN("oAccelerometerBias_", fpAccBiasX, fpAccBiasY, fpAccBiasZ);
    write3xN("oGyroscopeBias_", fpGyrBiasX, fpGyrBiasY, fpGyrBiasZ);
    write3xN("oAccelerometerScaleFactor_", fpAccScaleX, fpAccScaleY, fpAccScaleZ);
    write3xN("oGyroscopeScaleFactor_", fpGyrScaleX, fpGyrScaleY, fpGyrScaleZ);
    write3xN("oAccelerationNed_", fpAccNedX, fpAccNedY, fpAccNedZ);
    TinyMATWriter_endStruct(opMatWriter); // oState_

    TinyMATWriter_startStruct(opMatWriter, "oStateStd_");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fHeight_", fpHeightStd, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVelocityDown_", fpVelocityDownStd, iSize);
    write3xN("oAttitude_", fpAttStdX, fpAttStdY, fpAttStdZ);
    write3xN("oAccelerometerBias_", fpAccBiasStdX, fpAccBiasStdY, fpAccBiasStdZ);
    write3xN("oGyroscopeBias_", fpGyrBiasStdX, fpGyrBiasStdY, fpGyrBiasStdZ);
    write3xN("oAccelerometerScaleFactor_", fpAccScaleStdX, fpAccScaleStdY, fpAccScaleStdZ);
    write3xN("oGyroscopeScaleFactor_", fpGyrScaleStdX, fpGyrScaleStdY, fpGyrScaleStdZ);
    write3xN("oAccelerationNed_", fpAccNedStdX, fpAccNedStdY, fpAccNedStdZ);
    TinyMATWriter_endStruct(opMatWriter); // oStateStd_

    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eFilterMode_", upFilterMode, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "bVerticalChannelDiverged_", upVerticalChannelDiverged, iSize);
    TinyMATWriter_endStruct(opMatWriter); // kpStructName
  }
  catch (const std::bad_alloc&)
  {
    cout << "Failed to allocate memory. Skipping the struct " << kpStructName << "..." << endl;
  }

  delete[] fpHeight;
  delete[] fpVelocityDown;
  delete[] fpQuatW;
  delete[] fpQuatX;
  delete[] fpQuatY;
  delete[] fpQuatZ;
  delete[] fpAccBiasX;
  delete[] fpAccBiasY;
  delete[] fpAccBiasZ;
  delete[] fpGyrBiasX;
  delete[] fpGyrBiasY;
  delete[] fpGyrBiasZ;
  delete[] fpAccScaleX;
  delete[] fpAccScaleY;
  delete[] fpAccScaleZ;
  delete[] fpGyrScaleX;
  delete[] fpGyrScaleY;
  delete[] fpGyrScaleZ;
  delete[] fpAccNedX;
  delete[] fpAccNedY;
  delete[] fpAccNedZ;
  delete[] fpHeightStd;
  delete[] fpVelocityDownStd;
  delete[] fpAttStdX;
  delete[] fpAttStdY;
  delete[] fpAttStdZ;
  delete[] fpAccBiasStdX;
  delete[] fpAccBiasStdY;
  delete[] fpAccBiasStdZ;
  delete[] fpGyrBiasStdX;
  delete[] fpGyrBiasStdY;
  delete[] fpGyrBiasStdZ;
  delete[] fpAccScaleStdX;
  delete[] fpAccScaleStdY;
  delete[] fpAccScaleStdZ;
  delete[] fpGyrScaleStdX;
  delete[] fpGyrScaleStdY;
  delete[] fpGyrScaleStdZ;
  delete[] fpAccNedStdX;
  delete[] fpAccNedStdY;
  delete[] fpAccNedStdZ;
  delete[] upTimestampUs;
  delete[] upFilterMode;
  delete[] upVerticalChannelDiverged;
}

void writePortImuDataAfterMonitorToMat(TinyMATWriterFile* opMatWriter, std::vector<CImuDataAfterMonitor>& orPortDataContainer)
{
  using EScalarSignals = NImuMonitorApi::COutputData::EScalarSignals;

  float* fpSpecForceX{ nullptr };
  uint64_t* upTimestampUsSpecForceX{ nullptr };
  uint8_t* upSensorIdSpecForceX{ nullptr };
  uint8_t* upDetectionResultsSpecForceX{ nullptr };
  uint8_t* upIsolationResultsSpecForceX{ nullptr };
  uint8_t* upIsolatedSensorSpecForceX{ nullptr };
  uint8_t* upValidSpecForceX{ nullptr };

  float* fpSpecForceY{ nullptr };
  uint64_t* upTimestampUsSpecForceY{ nullptr };
  uint8_t* upSensorIdSpecForceY{ nullptr };
  uint8_t* upDetectionResultsSpecForceY{ nullptr };
  uint8_t* upIsolationResultsSpecForceY{ nullptr };
  uint8_t* upIsolatedSensorSpecForceY{ nullptr };
  uint8_t* upValidSpecForceY{ nullptr };

  float* fpSpecForceZ{ nullptr };
  uint64_t* upTimestampUsSpecForceZ{ nullptr };
  uint8_t* upSensorIdSpecForceZ{ nullptr };
  uint8_t* upDetectionResultsSpecForceZ{ nullptr };
  uint8_t* upIsolationResultsSpecForceZ{ nullptr };
  uint8_t* upIsolatedSensorSpecForceZ{ nullptr };
  uint8_t* upValidSpecForceZ{ nullptr };

  float* fpAngRateX{ nullptr };
  uint64_t* upTimestampUsAngRateX{ nullptr };
  uint8_t* upSensorIdAngRateX{ nullptr };
  uint8_t* upDetectionResultsAngRateX{ nullptr };
  uint8_t* upIsolationResultsAngRateX{ nullptr };
  uint8_t* upIsolatedSensorAngRateX{ nullptr };
  uint8_t* upValidAngRateX{ nullptr };

  float* fpAngRateY{ nullptr };
  uint64_t* upTimestampUsAngRateY{ nullptr };
  uint8_t* upSensorIdAngRateY{ nullptr };
  uint8_t* upDetectionResultsAngRateY{ nullptr };
  uint8_t* upIsolationResultsAngRateY{ nullptr };
  uint8_t* upIsolatedSensorAngRateY{ nullptr };
  uint8_t* upValidAngRateY{ nullptr };

  float* fpAngRateZ{ nullptr };
  uint64_t* upTimestampUsAngRateZ{ nullptr };
  uint8_t* upSensorIdAngRateZ{ nullptr };
  uint8_t* upDetectionResultsAngRateZ{ nullptr };
  uint8_t* upIsolationResultsAngRateZ{ nullptr };
  uint8_t* upIsolatedSensorAngRateZ{ nullptr };
  uint8_t* upValidAngRateZ{ nullptr };

  try
  {
    const size_t uSize = orPortDataContainer.size();
    fpSpecForceX = new float[uSize];
    upTimestampUsSpecForceX = new uint64_t[uSize];
    upSensorIdSpecForceX = new uint8_t[uSize];
    upDetectionResultsSpecForceX = new uint8_t[uSize];
    upIsolationResultsSpecForceX = new uint8_t[uSize];
    upIsolatedSensorSpecForceX = new uint8_t[uSize];
    upValidSpecForceX = new uint8_t[uSize];

    fpSpecForceY = new float[uSize];
    upTimestampUsSpecForceY = new uint64_t[uSize];
    upSensorIdSpecForceY = new uint8_t[uSize];
    upDetectionResultsSpecForceY = new uint8_t[uSize];
    upIsolationResultsSpecForceY = new uint8_t[uSize];
    upIsolatedSensorSpecForceY = new uint8_t[uSize];
    upValidSpecForceY = new uint8_t[uSize];

    fpSpecForceZ = new float[uSize];
    upTimestampUsSpecForceZ = new uint64_t[uSize];
    upSensorIdSpecForceZ = new uint8_t[uSize];
    upDetectionResultsSpecForceZ = new uint8_t[uSize];
    upIsolationResultsSpecForceZ = new uint8_t[uSize];
    upIsolatedSensorSpecForceZ = new uint8_t[uSize];
    upValidSpecForceZ = new uint8_t[uSize];

    fpAngRateX = new float[uSize];
    upTimestampUsAngRateX = new uint64_t[uSize];
    upSensorIdAngRateX = new uint8_t[uSize];
    upDetectionResultsAngRateX = new uint8_t[uSize];
    upIsolationResultsAngRateX = new uint8_t[uSize];
    upIsolatedSensorAngRateX = new uint8_t[uSize];
    upValidAngRateX = new uint8_t[uSize];

    fpAngRateY = new float[uSize];
    upTimestampUsAngRateY = new uint64_t[uSize];
    upSensorIdAngRateY = new uint8_t[uSize];
    upDetectionResultsAngRateY = new uint8_t[uSize];
    upIsolationResultsAngRateY = new uint8_t[uSize];
    upIsolatedSensorAngRateY = new uint8_t[uSize];
    upValidAngRateY = new uint8_t[uSize];

    fpAngRateZ = new float[uSize];
    upTimestampUsAngRateZ = new uint64_t[uSize];
    upSensorIdAngRateZ = new uint8_t[uSize];
    upDetectionResultsAngRateZ = new uint8_t[uSize];
    upIsolationResultsAngRateZ = new uint8_t[uSize];
    upIsolatedSensorAngRateZ = new uint8_t[uSize];
    upValidAngRateZ = new uint8_t[uSize];

    for (uint64_t uIndex = 0; uIndex < uSize; uIndex++)
    {
      auto& orData = orPortDataContainer[uIndex];

      const auto& oSfx = orData.GetSignal(EScalarSignals::eSpecificForceX);
      fpSpecForceX[uIndex] = oSfx.fSignal_;
      upTimestampUsSpecForceX[uIndex] = oSfx.uTimestampUs_;
      upSensorIdSpecForceX[uIndex] = static_cast<uint8_t>(oSfx.eSensorId_);
      upDetectionResultsSpecForceX[uIndex] = static_cast<uint8_t>(oSfx.eDetectionResults_);
      upIsolationResultsSpecForceX[uIndex] = static_cast<uint8_t>(oSfx.eIsolationResults_);
      upIsolatedSensorSpecForceX[uIndex] = static_cast<uint8_t>(oSfx.eIsolatedSensor_);
      upValidSpecForceX[uIndex] = oSfx.bValid_ ? 1U : 0U;

      const auto& oSfy = orData.GetSignal(EScalarSignals::eSpecificForceY);
      fpSpecForceY[uIndex] = oSfy.fSignal_;
      upTimestampUsSpecForceY[uIndex] = oSfy.uTimestampUs_;
      upSensorIdSpecForceY[uIndex] = static_cast<uint8_t>(oSfy.eSensorId_);
      upDetectionResultsSpecForceY[uIndex] = static_cast<uint8_t>(oSfy.eDetectionResults_);
      upIsolationResultsSpecForceY[uIndex] = static_cast<uint8_t>(oSfy.eIsolationResults_);
      upIsolatedSensorSpecForceY[uIndex] = static_cast<uint8_t>(oSfy.eIsolatedSensor_);
      upValidSpecForceY[uIndex] = oSfy.bValid_ ? 1U : 0U;

      const auto& oSfz = orData.GetSignal(EScalarSignals::eSpecificForceZ);
      fpSpecForceZ[uIndex] = oSfz.fSignal_;
      upTimestampUsSpecForceZ[uIndex] = oSfz.uTimestampUs_;
      upSensorIdSpecForceZ[uIndex] = static_cast<uint8_t>(oSfz.eSensorId_);
      upDetectionResultsSpecForceZ[uIndex] = static_cast<uint8_t>(oSfz.eDetectionResults_);
      upIsolationResultsSpecForceZ[uIndex] = static_cast<uint8_t>(oSfz.eIsolationResults_);
      upIsolatedSensorSpecForceZ[uIndex] = static_cast<uint8_t>(oSfz.eIsolatedSensor_);
      upValidSpecForceZ[uIndex] = oSfz.bValid_ ? 1U : 0U;

      const auto& oArx = orData.GetSignal(EScalarSignals::eAngularRateX);
      fpAngRateX[uIndex] = oArx.fSignal_;
      upTimestampUsAngRateX[uIndex] = oArx.uTimestampUs_;
      upSensorIdAngRateX[uIndex] = static_cast<uint8_t>(oArx.eSensorId_);
      upDetectionResultsAngRateX[uIndex] = static_cast<uint8_t>(oArx.eDetectionResults_);
      upIsolationResultsAngRateX[uIndex] = static_cast<uint8_t>(oArx.eIsolationResults_);
      upIsolatedSensorAngRateX[uIndex] = static_cast<uint8_t>(oArx.eIsolatedSensor_);
      upValidAngRateX[uIndex] = oArx.bValid_ ? 1U : 0U;

      const auto& oAry = orData.GetSignal(EScalarSignals::eAngularRateY);
      fpAngRateY[uIndex] = oAry.fSignal_;
      upTimestampUsAngRateY[uIndex] = oAry.uTimestampUs_;
      upSensorIdAngRateY[uIndex] = static_cast<uint8_t>(oAry.eSensorId_);
      upDetectionResultsAngRateY[uIndex] = static_cast<uint8_t>(oAry.eDetectionResults_);
      upIsolationResultsAngRateY[uIndex] = static_cast<uint8_t>(oAry.eIsolationResults_);
      upIsolatedSensorAngRateY[uIndex] = static_cast<uint8_t>(oAry.eIsolatedSensor_);
      upValidAngRateY[uIndex] = oAry.bValid_ ? 1U : 0U;

      const auto& oArz = orData.GetSignal(EScalarSignals::eAngularRateZ);
      fpAngRateZ[uIndex] = oArz.fSignal_;
      upTimestampUsAngRateZ[uIndex] = oArz.uTimestampUs_;
      upSensorIdAngRateZ[uIndex] = static_cast<uint8_t>(oArz.eSensorId_);
      upDetectionResultsAngRateZ[uIndex] = static_cast<uint8_t>(oArz.eDetectionResults_);
      upIsolationResultsAngRateZ[uIndex] = static_cast<uint8_t>(oArz.eIsolationResults_);
      upIsolatedSensorAngRateZ[uIndex] = static_cast<uint8_t>(oArz.eIsolatedSensor_);
      upValidAngRateZ[uIndex] = oArz.bValid_ ? 1U : 0U;
    }

    const auto iSize = static_cast<int32_t>(orPortDataContainer.size());
    TinyMATWriter_startStruct(opMatWriter, "PortImuDataAfterMonitor");

    TinyMATWriter_startStruct(opMatWriter, "oSpecificForceX_");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSignal_", fpSpecForceX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUsSpecForceX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eSensorId_", upSensorIdSpecForceX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eDetectionResults_", upDetectionResultsSpecForceX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolationResults_", upIsolationResultsSpecForceX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolatedSensor_", upIsolatedSensorSpecForceX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "bValid_", upValidSpecForceX, iSize);
    TinyMATWriter_endStruct(opMatWriter);

    TinyMATWriter_startStruct(opMatWriter, "oSpecificForceY_");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSignal_", fpSpecForceY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUsSpecForceY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eSensorId_", upSensorIdSpecForceY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eDetectionResults_", upDetectionResultsSpecForceY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolationResults_", upIsolationResultsSpecForceY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolatedSensor_", upIsolatedSensorSpecForceY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "bValid_", upValidSpecForceY, iSize);
    TinyMATWriter_endStruct(opMatWriter);

    TinyMATWriter_startStruct(opMatWriter, "oSpecificForceZ_");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSignal_", fpSpecForceZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUsSpecForceZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eSensorId_", upSensorIdSpecForceZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eDetectionResults_", upDetectionResultsSpecForceZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolationResults_", upIsolationResultsSpecForceZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolatedSensor_", upIsolatedSensorSpecForceZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "bValid_", upValidSpecForceZ, iSize);
    TinyMATWriter_endStruct(opMatWriter);

    TinyMATWriter_startStruct(opMatWriter, "oAngularRateX_");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSignal_", fpAngRateX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUsAngRateX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eSensorId_", upSensorIdAngRateX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eDetectionResults_", upDetectionResultsAngRateX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolationResults_", upIsolationResultsAngRateX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolatedSensor_", upIsolatedSensorAngRateX, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "bValid_", upValidAngRateX, iSize);
    TinyMATWriter_endStruct(opMatWriter);

    TinyMATWriter_startStruct(opMatWriter, "oAngularRateY_");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSignal_", fpAngRateY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUsAngRateY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eSensorId_", upSensorIdAngRateY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eDetectionResults_", upDetectionResultsAngRateY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolationResults_", upIsolationResultsAngRateY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolatedSensor_", upIsolatedSensorAngRateY, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "bValid_", upValidAngRateY, iSize);
    TinyMATWriter_endStruct(opMatWriter);

    TinyMATWriter_startStruct(opMatWriter, "oAngularRateZ_");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSignal_", fpAngRateZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUsAngRateZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eSensorId_", upSensorIdAngRateZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eDetectionResults_", upDetectionResultsAngRateZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolationResults_", upIsolationResultsAngRateZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "eIsolatedSensor_", upIsolatedSensorAngRateZ, iSize);
    TinyMATWriter_writeVectorAsRow(opMatWriter, "bValid_", upValidAngRateZ, iSize);
    TinyMATWriter_endStruct(opMatWriter);

    TinyMATWriter_endStruct(opMatWriter); // PortImuDataAfterMonitor
  }
  catch (const std::bad_alloc&)
  {
    cout << "Failed to allocate memory. Skipping the struct PortImuDataAfterMonitor..." << endl;
  }

  delete[] fpSpecForceX;
  delete[] upTimestampUsSpecForceX;
  delete[] upSensorIdSpecForceX;
  delete[] upDetectionResultsSpecForceX;
  delete[] upIsolationResultsSpecForceX;
  delete[] upIsolatedSensorSpecForceX;
  delete[] upValidSpecForceX;
  delete[] fpSpecForceY;
  delete[] upTimestampUsSpecForceY;
  delete[] upSensorIdSpecForceY;
  delete[] upDetectionResultsSpecForceY;
  delete[] upIsolationResultsSpecForceY;
  delete[] upIsolatedSensorSpecForceY;
  delete[] upValidSpecForceY;
  delete[] fpSpecForceZ;
  delete[] upTimestampUsSpecForceZ;
  delete[] upSensorIdSpecForceZ;
  delete[] upDetectionResultsSpecForceZ;
  delete[] upIsolationResultsSpecForceZ;
  delete[] upIsolatedSensorSpecForceZ;
  delete[] upValidSpecForceZ;
  delete[] fpAngRateX;
  delete[] upTimestampUsAngRateX;
  delete[] upSensorIdAngRateX;
  delete[] upDetectionResultsAngRateX;
  delete[] upIsolationResultsAngRateX;
  delete[] upIsolatedSensorAngRateX;
  delete[] upValidAngRateX;
  delete[] fpAngRateY;
  delete[] upTimestampUsAngRateY;
  delete[] upSensorIdAngRateY;
  delete[] upDetectionResultsAngRateY;
  delete[] upIsolationResultsAngRateY;
  delete[] upIsolatedSensorAngRateY;
  delete[] upValidAngRateY;
  delete[] fpAngRateZ;
  delete[] upTimestampUsAngRateZ;
  delete[] upSensorIdAngRateZ;
  delete[] upDetectionResultsAngRateZ;
  delete[] upIsolationResultsAngRateZ;
  delete[] upIsolatedSensorAngRateZ;
  delete[] upValidAngRateZ;
}
