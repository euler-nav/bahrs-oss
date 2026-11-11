/// @file SerialProtocolToMat.cpp
/// @brief Implementation of the SerialProtocolToMat tool. This file contains the 'main' function. Program execution begins and ends there.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include "tinymatwriter.h"
#include "CSerialProtocol.h"
#include "Crc32.h"

using namespace std;

static vector<CSerialProtocol::SInertialData> oInertialData;
static vector<CSerialProtocol::SNavigationData> oNavigationData;
static vector<CSerialProtocol::SAccuracyData> oAccuraryData;
static vector<CSerialProtocol::STimeOfNavigationData> oTimeOfNavigationData;
static vector<CSerialProtocol::STimeOfInertialData> oTimeOfInertialData;
static vector<CSerialProtocol::STimeOfLatestSyncPulse> oTimeOfLatestSyncPulse;

/// @brief Write messages to MAT file.
/// @param oName MAT-file name
void CreateMatFile(string oName);

/// @brief Write inertial data messages to the MAT-file.
/// @param opMatWriter Pointer to the MAT-file writer
/// @param orInertialData Reference to a vector of message payloads.
static void writeInertialDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::SInertialData>& orInertialData);

/// @brief Write navigation data messages to the MAT-file.
/// @param opMatWriter Pointer to the MAT-file writer
/// @param orNavigationData Reference to a vector of message payloads.
static void writeNavigationDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::SNavigationData>& orNavigationData);

/// @brief Write accuracy data messages to the MAT-file.
/// @param opMatWriter Pointer to the MAT-file writer
/// @param orAccuracyData Reference to a vector of message payloads.
static void writeAccuracyDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::SAccuracyData>& orAccuracyData);

/// @brief Write time of navigation data messages to the MAT-file.
/// @param opMatWriter Pointer to the MAT-file writer
/// @param orTimeOfNavigationData Reference to a vector of message payloads.
static void writeTimeOfNavigationDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::STimeOfNavigationData>& orTimeOfNavigationData);

/// @brief Write time of inertial data messages to the MAT-file.
/// @param opMatWriter Pointer to the MAT-file writer
/// @param orTimeOfInertialData Reference to a vector of message payloads.
static void writeTimeOfInertialDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::STimeOfInertialData>& orTimeOfInertialData);

/// @brief Write time of the latest sync pulse messages to the MAT-file.
/// @param opMatWriter Pointer to the MAT-file writer
/// @param orTimeOfLatestSyncPulse Reference to a vector of message payloads.
static void writeTimeOfLatestSyncPulseToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::STimeOfLatestSyncPulse>& orTimeOfLatestSyncPulse);

/// @brief Save message payload to a vector.
/// @tparam tMessagePayloadType Message struct type
/// @tparam tMessageType Payload struct type
/// @param kurFileLength Total length of the log file.
/// @param kupFileData Pointer to a byte array read from the log file.
/// @param urPositionInFile Current position in the byte array.
/// @param orVector Reference to the destination vector.
/// @return True is we reached the end of the byte array, false otherwise.
template<typename tMessageType, typename tMessagePayloadType> bool SaveToVector(const size_t& kurFileLength,
                                                                                const uint8_t* kupFileData,
                                                                                size_t& urPositionInFile,
                                                                                std::vector<tMessagePayloadType>& orVector)
{
  bool bStopReadingFile = false;
  const unsigned uCrcSize = 4U;

  if ((kurFileLength - urPositionInFile) < sizeof(tMessageType))
  {
    bStopReadingFile = true;
  }
  else
  {
    const uint8_t* kupMessageStartAddress = kupFileData + urPositionInFile;
    const tMessageType* opMessage = reinterpret_cast<const tMessageType*>(kupMessageStartAddress);
    unsigned uLengthInWords = static_cast<uint8_t>((sizeof(tMessageType) - uCrcSize) / uCrcSize);

    if (opMessage->uCrc_ == Crc32(reinterpret_cast<const uint32_t*>(kupMessageStartAddress), uLengthInWords))
    {
      if (opMessage->oHeader_.uVersion_ == CSerialProtocol::uVersion_)
      {
        const uint8_t* kupPayloadStartAddress = kupFileData + urPositionInFile + sizeof(CSerialProtocol::SMessageHeader);
        const tMessagePayloadType* tpPayload = reinterpret_cast<const tMessagePayloadType*>(kupPayloadStartAddress);
        orVector.push_back(*tpPayload);
      }
      else
      {
        cout << "Protocol version mismatch!!! Position in file " << urPositionInFile << ". Skipping the message..." << endl;
      }

      urPositionInFile += sizeof(tMessageType);
    }
    else
    {
      cout << "CRC is not correct for message type " << static_cast<uint16_t>(opMessage->oHeader_.uMsgType_) << ". Position in file is " << urPositionInFile << endl;
      urPositionInFile += 2U;
    }
  }

  return bStopReadingFile;
}

int main(int argc, char* argv[])
{
  ifstream oInputFile;
  uint8_t* upBuff;
  size_t uFileLength = 0U;
  string oFilename;
  string oMatFilename;
  
  if (argc == 3)
  {
    oFilename.append(argv[1]);
    oMatFilename.append(argv[2]);   
  }
  else
  {
    cout << "Please enter file name for read\n";
    cin >> oFilename;
    cout << "Please enter file name for convert\n";
    cin >> oMatFilename;
  }
  
  oInputFile.open(oFilename, ios::binary);

  if (oInputFile.is_open())
  {   
    oInputFile.seekg(0, ios::end);
    uFileLength = oInputFile.tellg();
    oInputFile.seekg(0, ios::beg);

    try
    {
      upBuff = new uint8_t[uFileLength];
    }
    catch (std::bad_alloc)
    {
      cout << "Cannot load the input file into RAM. Not enough memory. The programm will terminate." << endl;
      oInputFile.close();
      return 4;
    }

    oInputFile.read(reinterpret_cast<char*>(upBuff), uFileLength);    
    oInputFile.close();
  }
  else
  {
    return 1;
  }

  // Parse the input file
  size_t uIndex = 0U;

  while (uIndex <= (uFileLength - sizeof(CSerialProtocol::SMessageHeader)))
  {
    CSerialProtocol::SMessageHeader* opMessageHeader = reinterpret_cast<CSerialProtocol::SMessageHeader*>(upBuff + uIndex);
    
    if ((CSerialProtocol::uMarker1_ == opMessageHeader->uMarker1_) && (CSerialProtocol::uMarker2_ == opMessageHeader->uMarker2_))
    {
      CSerialProtocol::EMessageIds eMessageType = static_cast<CSerialProtocol::EMessageIds>(opMessageHeader->uMsgType_);
      bool bNeedToBreak = false;

      switch (eMessageType)
      {
        case CSerialProtocol::EMessageIds::eInertialData:
        {
          bNeedToBreak = SaveToVector<CSerialProtocol::SInertialDataMessage, CSerialProtocol::SInertialData>(uFileLength, upBuff, uIndex, oInertialData);
          break;
        }
        case CSerialProtocol::EMessageIds::eNavigationData:
        {
          bNeedToBreak = SaveToVector<CSerialProtocol::SNavigationDataMessage, CSerialProtocol::SNavigationData>(uFileLength, upBuff, uIndex, oNavigationData);
          break;
        }
        case CSerialProtocol::EMessageIds::eAccuracy:
        {
          bNeedToBreak = SaveToVector<CSerialProtocol::SAccuracyDataMessage, CSerialProtocol::SAccuracyData>(uFileLength, upBuff, uIndex, oAccuraryData);
          break;
        }
        case CSerialProtocol::EMessageIds::eTimeOfNavigationData:
        {
          bNeedToBreak = SaveToVector<CSerialProtocol::STimeOfNavigationDataMessage, CSerialProtocol::STimeOfNavigationData>(uFileLength, upBuff, uIndex, oTimeOfNavigationData);
          break;
        }
        case CSerialProtocol::EMessageIds::eTimeOfInertialData:
        {
          bNeedToBreak = SaveToVector<CSerialProtocol::STimeOfInertialDataMessage, CSerialProtocol::STimeOfInertialData>(uFileLength, upBuff, uIndex, oTimeOfInertialData);
          break;
        }
        case CSerialProtocol::EMessageIds::eTimeOfSyncPulse:
        {
          bNeedToBreak = SaveToVector<CSerialProtocol::STimeOfLatestSyncPulseMessage, CSerialProtocol::STimeOfLatestSyncPulse>(uFileLength, upBuff, uIndex, oTimeOfLatestSyncPulse);
          break;
        }
        default:
          ++uIndex;
          break;
      }

      if (true == bNeedToBreak)
      {
        break;
      }
    }
    else
    {
      ++uIndex;
    }
  }

  delete [] upBuff;

  CreateMatFile(oMatFilename);

  return 0;
}


void CreateMatFile(string oName)
{
  TinyMATWriterFile* opMat = TinyMATWriter_open(oName.data());

  if (nullptr != opMat)
  {
    writeInertialDataToMat(opMat, oInertialData);
    writeNavigationDataToMat(opMat, oNavigationData);
    writeAccuracyDataToMat(opMat, oAccuraryData);
    writeTimeOfNavigationDataToMat(opMat, oTimeOfNavigationData);
    writeTimeOfInertialDataToMat(opMat, oTimeOfInertialData);
    writeTimeOfLatestSyncPulseToMat(opMat, oTimeOfLatestSyncPulse);
  }
}

void writeInertialDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::SInertialData>& orInertialData)
{

  int16_t* ipSpecificForceX { nullptr };
  int16_t* ipSpecificForceY { nullptr };
  int16_t* ipSpecificForceZ { nullptr };
  int16_t* ipAngularRateX { nullptr };
  int16_t* ipAngularRateY { nullptr };
  int16_t* ipAngularRateZ { nullptr };
  uint8_t* upSequenceCounter { nullptr };
  uint16_t* upHealth { nullptr };

  try
  {
    ipSpecificForceX = new int16_t[orInertialData.size()];
    ipSpecificForceY = new int16_t[orInertialData.size()];
    ipSpecificForceZ = new int16_t[orInertialData.size()];
    ipAngularRateX = new int16_t[orInertialData.size()];
    ipAngularRateY = new int16_t[orInertialData.size()];
    ipAngularRateZ = new int16_t[orInertialData.size()];
    upSequenceCounter = new uint8_t[orInertialData.size()];
    upHealth = new uint16_t[orInertialData.size()];

    for (uint64_t uIndex = 0; uIndex < orInertialData.size(); uIndex++)
    {
      ipSpecificForceX[uIndex] = orInertialData[uIndex].iSpecificForceX_;
      ipSpecificForceY[uIndex] = orInertialData[uIndex].iSpecificForceY_;
      ipSpecificForceZ[uIndex] = orInertialData[uIndex].iSpecificForceZ_;

      ipAngularRateX[uIndex] = orInertialData[uIndex].iAngularRateX_;
      ipAngularRateY[uIndex] = orInertialData[uIndex].iAngularRateY_;
      ipAngularRateZ[uIndex] = orInertialData[uIndex].iAngularRateZ_;

      upSequenceCounter[uIndex] = orInertialData[uIndex].uSequenceCounter_;
      upHealth[uIndex] = orInertialData[uIndex].uHealthInfo_;
    }

    TinyMATWriter_startStruct(opMatWriter, "SInertialDataMessage");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iSpecificForceX_", ipSpecificForceX, static_cast<int32_t>(orInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iSpecificForceY_", ipSpecificForceY, static_cast<int32_t>(orInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iSpecificForceZ_", ipSpecificForceZ, static_cast<int32_t>(orInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iAngularRateX_", ipAngularRateX, static_cast<int32_t>(orInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iAngularRateY_", ipAngularRateY, static_cast<int32_t>(orInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iAngularRateZ_", ipAngularRateZ, static_cast<int32_t>(orInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uSequenceCounter_", upSequenceCounter, static_cast<int32_t>(orInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uHealthInfo_", upHealth, static_cast<int32_t>(orInertialData.size()));
    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct SInertialDataMessage..." << endl;
  }

  delete [] ipSpecificForceX;
  delete [] ipSpecificForceY;
  delete [] ipSpecificForceZ;
  delete [] ipAngularRateX;
  delete [] ipAngularRateY;
  delete [] ipAngularRateZ;
  delete [] upSequenceCounter;
  delete [] upHealth;
}

void writeNavigationDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::SNavigationData>& orNavigationData)
{
  //NavigationData variables
  uint16_t* upPressureHeight { nullptr };
  int16_t* ipVelocityDown { nullptr };
  int16_t* ipRoll { nullptr };
  int16_t* ipPitch { nullptr };
  uint16_t* upMagneticHeading { nullptr };
  uint16_t* upHealth { nullptr };
  uint8_t* upSequenceCounter { nullptr };

  try
  {
    upSequenceCounter = new uint8_t[orNavigationData.size()];
    upPressureHeight = new uint16_t[orNavigationData.size()];
    ipVelocityDown = new int16_t[orNavigationData.size()];
    ipRoll = new int16_t[orNavigationData.size()];
    ipPitch = new int16_t[orNavigationData.size()];
    upMagneticHeading = new uint16_t[orNavigationData.size()];
    upHealth = new uint16_t[orNavigationData.size()];

    for (uint64_t uIndex = 0; uIndex < orNavigationData.size(); uIndex++)
    {
      upSequenceCounter[uIndex] = orNavigationData[uIndex].uSequenceCounter_;
      upPressureHeight[uIndex] = orNavigationData[uIndex].uPressureHeight_;
      ipVelocityDown[uIndex] = orNavigationData[uIndex].iVelocityDown_;
      ipRoll[uIndex] = orNavigationData[uIndex].iRoll_;
      ipPitch[uIndex] = orNavigationData[uIndex].iPitch_;
      upMagneticHeading[uIndex] = orNavigationData[uIndex].uMagneticHeading_;
      upHealth[uIndex] = orNavigationData[uIndex].uHealthInfo_;
    }

    TinyMATWriter_startStruct(opMatWriter, "SNavigationDataMessage");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uSequenceCounter_", upSequenceCounter, static_cast<int32_t>(orNavigationData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uPressureHeight_", upPressureHeight, static_cast<int32_t>(orNavigationData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iVelocityDown_", ipVelocityDown, static_cast<int32_t>(orNavigationData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iRoll_", ipRoll, static_cast<int32_t>(orNavigationData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "iPitch_", ipPitch, static_cast<int32_t>(orNavigationData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uMagneticHeading_", upMagneticHeading, static_cast<int32_t>(orNavigationData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uHealthInfo_", upHealth, static_cast<int32_t>(orNavigationData.size()));
    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct SNavigationDataMessage..." << endl;
  }

  delete [] upPressureHeight;
  delete [] ipVelocityDown;
  delete [] ipRoll;
  delete [] ipPitch;
  delete [] upMagneticHeading;
  delete [] upSequenceCounter;
  delete [] upHealth;
}

void writeAccuracyDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::SAccuracyData>& orAccuracyData)
{
  
  uint8_t* upSequenceCounter { nullptr };
  uint16_t* upAttitudeStdN { nullptr };
  uint16_t* upAttitudeStdE { nullptr };
  uint16_t* upMagneticHeadingStd { nullptr };
  uint64_t* upTimestampUs { nullptr };

  try
  {
    upSequenceCounter = new uint8_t[orAccuracyData.size()];
    upAttitudeStdN = new uint16_t[orAccuracyData.size()];
    upAttitudeStdE = new uint16_t[orAccuracyData.size()];
    upMagneticHeadingStd = new uint16_t[orAccuracyData.size()];
    upTimestampUs = new uint64_t[orAccuracyData.size()];

    for (uint64_t uIndex = 0; uIndex < orAccuracyData.size(); uIndex++)
    {
      upSequenceCounter[uIndex] = orAccuracyData[uIndex].uSequenceCounter_;
      upAttitudeStdN[uIndex] = orAccuracyData[uIndex].uAttitudeStdN_;
      upAttitudeStdE[uIndex] = orAccuracyData[uIndex].uAttitudeStdE_;
      upMagneticHeadingStd[uIndex] = orAccuracyData[uIndex].uMagneticHeadingStd_;
      upTimestampUs[uIndex] = orAccuracyData[uIndex].uTimestampUs_;
    }

    TinyMATWriter_startStruct(opMatWriter, "SAccuracyDataMessage");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uSequenceCounter_", upSequenceCounter, static_cast<int32_t>(orAccuracyData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uAttitudeStdN_", upAttitudeStdN, static_cast<int32_t>(orAccuracyData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uAttitudeStdE_", upAttitudeStdE, static_cast<int32_t>(orAccuracyData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uMagneticHeadingStd_", upMagneticHeadingStd, static_cast<int32_t>(orAccuracyData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orAccuracyData.size()));
    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct SAccuracyDataMessage..." << endl;
  }

  delete [] upAttitudeStdN;
  delete [] upAttitudeStdE;
  delete [] upMagneticHeadingStd;
  delete [] upSequenceCounter;
}

void writeTimeOfNavigationDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::STimeOfNavigationData>& orTimeOfNavigationData)
{
  uint8_t* upSequenceCounter { nullptr };
  uint8_t* upNavigationDataSequenceCounter { nullptr };
  uint64_t* upTimestampUs { nullptr };

  try
  {
    upSequenceCounter = new uint8_t[orTimeOfNavigationData.size()];
    upNavigationDataSequenceCounter = new uint8_t[orTimeOfNavigationData.size()];
    upTimestampUs = new uint64_t[orTimeOfNavigationData.size()];

    for (uint64_t uIndex = 0; uIndex < orTimeOfNavigationData.size(); uIndex++)
    {
      upSequenceCounter[uIndex] = orTimeOfNavigationData[uIndex].uSequenceCounter_;
      upNavigationDataSequenceCounter[uIndex] = orTimeOfNavigationData[uIndex].uNavigationDataSequenceCounter_;
      upTimestampUs[uIndex] = orTimeOfNavigationData[uIndex].uTimestampUs_;
    }

    TinyMATWriter_startStruct(opMatWriter, "STimeOfNavigationDataMessage");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uSequenceCounter_", upSequenceCounter, static_cast<int32_t>(orTimeOfNavigationData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uNavigationDataSequenceCounter_", upNavigationDataSequenceCounter, static_cast<int32_t>(orTimeOfNavigationData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orTimeOfNavigationData.size()));
    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct STimeOfNavigationDataMessage..." << endl;
  }

  delete [] upNavigationDataSequenceCounter;
  delete [] upTimestampUs;
  delete [] upSequenceCounter;
}

void writeTimeOfInertialDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::STimeOfInertialData>& orTimeOfInertialData)
{
  uint8_t* upSequenceCounter { nullptr };
  uint8_t* upInertialDataSequenceCounter { nullptr };
  uint64_t* upTimestampUs { nullptr };

  try
  {
    upSequenceCounter = new uint8_t[orTimeOfInertialData.size()];
    upInertialDataSequenceCounter = new uint8_t[orTimeOfInertialData.size()];
    upTimestampUs = new uint64_t[orTimeOfInertialData.size()];

    for (uint64_t uIndex = 0; uIndex < orTimeOfInertialData.size(); uIndex++)
    {
      upSequenceCounter[uIndex] = orTimeOfInertialData[uIndex].uSequenceCounter_;
      upInertialDataSequenceCounter[uIndex] = orTimeOfInertialData[uIndex].uInertialDataSequenceCounter_;
      upTimestampUs[uIndex] = orTimeOfInertialData[uIndex].uTimestampUs_;
    }

    TinyMATWriter_startStruct(opMatWriter, "STimeOfInertialDataMessage");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uSequenceCounter_", upSequenceCounter, static_cast<int32_t>(orTimeOfInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uInertialDataSequenceCounter_", upInertialDataSequenceCounter, static_cast<int32_t>(orTimeOfInertialData.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orTimeOfInertialData.size()));
    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct STimeOfNavigationDataMessage..." << endl;
  }

  delete [] upInertialDataSequenceCounter;
  delete [] upTimestampUs;
  delete [] upSequenceCounter;
}

void writeTimeOfLatestSyncPulseToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::STimeOfLatestSyncPulse>& orTimeOfLatestSyncPulse)
{
  uint8_t* upSequenceCounter { nullptr };
  uint64_t* upTimestampUs { nullptr };

  try
  {
    upSequenceCounter = new uint8_t[orTimeOfLatestSyncPulse.size()];
    upTimestampUs = new uint64_t[orTimeOfLatestSyncPulse.size()];

    for (uint64_t uIndex = 0; uIndex < orTimeOfLatestSyncPulse.size(); uIndex++)
    {
      upSequenceCounter[uIndex] = orTimeOfLatestSyncPulse[uIndex].uSequenceCounter_;
      upTimestampUs[uIndex] = orTimeOfLatestSyncPulse[uIndex].uTimestampUs_;
    }

    TinyMATWriter_startStruct(opMatWriter, "STimeOfLatestSyncPulse");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uSequenceCounter_", upSequenceCounter, static_cast<int32_t>(orTimeOfLatestSyncPulse.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orTimeOfLatestSyncPulse.size()));
    TinyMATWriter_endStruct(opMatWriter);
    TinyMATWriter_close(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct STimeOfLatestSyncPulseMessage..." << endl;
  }

  delete [] upTimestampUs;
  delete [] upSequenceCounter;
}

