/**
 * @file DebugOutputToMat.cpp
 * @brief Converter of binary debug data to MAT.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date 23.10.2026 at 14:50:40
 */

#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include "tinymatwriter.h"
#include "CSerialProtocol.h"
#include "CRte.h"
#include "Crc32.h"
#include "WriteUserDefinedDataPorts.h"

using namespace std;

static vector<CSerialProtocol::SRunnableCallEventData> oRunnableCallEventDataContainer;

static vector<SImuMeasurement> oPortDataImuInput1Container;
static vector<SBarometerMeasurement> oPortDataPressureInput1Container;
static vector<SMagneticMeasurement> oPortDataMagnetometerInput1Container;
static vector<SMagneticMeasurement> oPortDataMagnetometerInput2Container;
static vector<SMagneticMeasurement> oPortDataMagnetometerInput3Container;
static vector<SBarometerMeasurement> oPortDataPressureInput2Container;
static vector<SBarometerMeasurement> oPortDataPressureInput3Container;
static vector<SImuMeasurement> oPortDataImuInput2Container;
static vector<SImuMeasurement> oPortDataImuInput3Container;
static vector<STimeOfSyncPulse> oPortDataSyncPulseTimeContainer;
static vector<SMagneticMeasurement> oPortDataCompensatedMagnetometerData1Container;
static vector<SMagneticHeading> oPortDataMagneticHeading1Container;
static vector<SMagneticMeasurement> oPortDataCompensatedMagnetometerDataInVehicleFrame1Container;
static vector<SBarometerMeasurement> oPortDataSafePressureData1Container;
static vector<SBarometerMeasurement> oPortDataSafePressureData2Container;
static vector<SBarometerMeasurement> oPortDataSafePressureData3Container;
static vector<SAttitudeData> oPortDataVehicleAttitude1Container;
static vector<SAttitudeData> oPortDataVehicleAttitude2Container;
static vector<SAttitudeData> oPortDataVehicleAttitude3Container;
static vector<SBarometerMeasurement> oPortDataCompensatedPressureInput1Container;
static vector<SBarometerMeasurement> oPortDataCompensatedPressureInput2Container;
static vector<SBarometerMeasurement> oPortDataCompensatedPressureInput3Container;
static vector<SImuMeasurement> oPortDataImuInputFiltered1Container;
static vector<SImuMeasurement> oPortDataImuInputFiltered2Container;
static vector<SImuMeasurement> oPortDataImuInputFiltered3Container;
static vector<SMagneticMeasurement> oPortDataCompensatedMagnetometerData2Container;
static vector<SMagneticMeasurement> oPortDataCompensatedMagnetometerData3Container;
static vector<SMagneticHeading> oPortDataMagneticHeading2Container;
static vector<SMagneticHeading> oPortDataMagneticHeading3Container;
static vector<SMagneticMeasurement> oPortDataCompensatedMagnetometerDataInVehicleFrame2Container;
static vector<SMagneticMeasurement> oPortDataCompensatedMagnetometerDataInVehicleFrame3Container;



/**
 * @brief Write messages to MAT file.
 * @param oName MAT-file name 
*/
void CreateMatFile(string oName);

/**
 * @brief Handles port write event data detected in the byte array.
 * The function extracts port write event message, checks CRC, and saves extracted data into a vector of structs.
 * Finally, the funtion increments the position in the byte array.
 * @param upMessageStartAddress Pointer to the beginning of the message.
 * @param uFileLength Length of the log file in bytes.
 * @param urPositionInFile Current position in the file/byte array.
*/
static void handlePortWriteEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile);

/**
 * @brief Handle runnable call event.
 * Extracts debug data from a byte array, checks CRC, and calls a runnable.
 * @param upMessageStartAddress Pointer to the first byte of the debug message.
 * @param uFileLength Length of the log file.
 * @param urPositionInFile Current position in the file.
*/
static void handleRunnableCallEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile);

/**
 * @brief Extracts auxiliary data from the port write event message.
 * @param upMessageStartAddress Pointer to the beginning of the message.
 * @param orPartialData Reference to output struct.
*/
static void getWriteToPortEventPartialData(const uint8_t* upMessageStartAddress, CSerialProtocol::SWriteToPortEventPartialData& orPartialData);

/**
 * @brief Extract a runnable call debug message from a byte array loaded to RAM.
 * @param upMessageStartAddress Pointer to the first byte of the debug message.
 * @param orMessage Reference to a structure to be filled.
*/
static void getRunnableCallEventMessage(const uint8_t* upMessageStartAddress, CSerialProtocol::SRunnableCallEventDebugMessage& orMessage);

/**
 * @brief Write runnable call event data to MAT file.
 * @param opMatWriter Pointer to the MAT file writer object.
 * @param orRunnableCallEventDataContainer Reference to the container with runnable call event data.
*/
static void writeRunnableCallEventDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::SRunnableCallEventData>& orRunnableCallEventDataContainer);

static void writePortImuInput1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer);
static void writePortPressureInput1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortMagnetometerInput1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);
static void writePortMagnetometerInput2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);
static void writePortMagnetometerInput3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);
static void writePortPressureInput2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortPressureInput3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortImuInput2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer);
static void writePortImuInput3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer);
static void writePortSyncPulseTimeToMat(TinyMATWriterFile* opMatWriter, std::vector<STimeOfSyncPulse>& orPortDataContainer);
static void writePortCompensatedMagnetometerData1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);
static void writePortMagneticHeading1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticHeading>& orPortDataContainer);
static void writePortCompensatedMagnetometerDataInVehicleFrame1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);
static void writePortSafePressureData1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortSafePressureData2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortSafePressureData3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortVehicleAttitude1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SAttitudeData>& orPortDataContainer);
static void writePortVehicleAttitude2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SAttitudeData>& orPortDataContainer);
static void writePortVehicleAttitude3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SAttitudeData>& orPortDataContainer);
static void writePortCompensatedPressureInput1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortCompensatedPressureInput2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortCompensatedPressureInput3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer);
static void writePortImuInputFiltered1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer);
static void writePortImuInputFiltered2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer);
static void writePortImuInputFiltered3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer);
static void writePortCompensatedMagnetometerData2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);
static void writePortCompensatedMagnetometerData3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);
static void writePortMagneticHeading2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticHeading>& orPortDataContainer);
static void writePortMagneticHeading3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticHeading>& orPortDataContainer);
static void writePortCompensatedMagnetometerDataInVehicleFrame2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);
static void writePortCompensatedMagnetometerDataInVehicleFrame3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer);


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

      switch (eMessageType)
      {
      case CSerialProtocol::EMessageIds::eDebugEventWriteToPort:
      {
        handlePortWriteEvent(upBuff + uIndex, uFileLength, uIndex);
        break;
      }
      case CSerialProtocol::EMessageIds::eDebugEventRunnableCall:
      {
        handleRunnableCallEvent(upBuff + uIndex, uFileLength, uIndex);
        break;
      }
      default:
      {
        ++uIndex;
        break;
      }
      }
    }
    else
    {
      ++uIndex;
    }
  }

  delete[] upBuff;

  CreateMatFile(oMatFilename);

  return 0;
}

void handlePortWriteEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile)
{
  CSerialProtocol::SWriteToPortEventPartialData oEventPartialData;
  unsigned uDataLen = 0U, uMessageLen = 0U;
  bool bStatus = false;

  if ((urPositionInFile + sizeof(CSerialProtocol::SWriteToPortEventPartialData)) < uFileLength)
  {
    getWriteToPortEventPartialData(upMessageStartAddress, oEventPartialData);

    uMessageLen += sizeof(CSerialProtocol::SMessageHeader);
    uMessageLen += sizeof(CSerialProtocol::SWriteToPortEventPartialData);
    uMessageLen += oEventPartialData.uDataLen_;
    uMessageLen += ((PROTOCOL_WORD_LEN - (uMessageLen % PROTOCOL_WORD_LEN))); // pad to multiple of 32 bit words
    uMessageLen += sizeof(CSerialProtocol::CrcType_t);

    if ((urPositionInFile + uMessageLen) < uFileLength)
    {
      uint32_t uReceivedCrc = *reinterpret_cast<const uint32_t*>(upMessageStartAddress + uMessageLen - sizeof(CSerialProtocol::CrcType_t));
      size_t uLengthInWords = (uMessageLen - sizeof(CSerialProtocol::CrcType_t)) / PROTOCOL_WORD_LEN;

      if (uReceivedCrc == Crc32(reinterpret_cast<const uint32_t*>(upMessageStartAddress), uLengthInWords))
      {
        bStatus = true;
      }
      else if (0U == uReceivedCrc)
      {
        // In some cases, the CRC may be missing and represented as 0. We will process such messages, but print a warning.
        bStatus = true;
        std::cout << "Warning: CRC is missing for a write to port event message. Position in file is " << urPositionInFile << std::endl;
      }
      else
      {
        std::cout << "CRC failed for a write to port event message. Position in file is " << urPositionInFile << std::endl;
      }
    }
  }


  if (true == bStatus)
  {
    CRte::EPortIds ePortId = static_cast<CRte::EPortIds>(oEventPartialData.uPortId_);
    unsigned uDataOffset = sizeof(CSerialProtocol::SMessageHeader) + sizeof(CSerialProtocol::SWriteToPortEventPartialData);
    std::vector<uint8_t> oBytes(upMessageStartAddress + uDataOffset, upMessageStartAddress + uDataOffset + oEventPartialData.uDataLen_);

    switch (ePortId)
    {
    
    case CRte::EPortIds::eImuInput1:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataImuInput1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::ePressureInput1:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataPressureInput1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagnetometerInput1:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataMagnetometerInput1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagnetometerInput2:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataMagnetometerInput2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagnetometerInput3:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataMagnetometerInput3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::ePressureInput2:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataPressureInput2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::ePressureInput3:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataPressureInput3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInput2:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataImuInput2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInput3:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataImuInput3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eSyncPulseTime:
      {
        STimeOfSyncPulse oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataSyncPulseTimeContainer.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerData1:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedMagnetometerData1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagneticHeading1:
      {
        SMagneticHeading oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataMagneticHeading1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame1:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedMagnetometerDataInVehicleFrame1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eSafePressureData1:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataSafePressureData1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eSafePressureData2:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataSafePressureData2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eSafePressureData3:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataSafePressureData3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eVehicleAttitude1:
      {
        SAttitudeData oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataVehicleAttitude1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eVehicleAttitude2:
      {
        SAttitudeData oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataVehicleAttitude2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eVehicleAttitude3:
      {
        SAttitudeData oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataVehicleAttitude3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedPressureInput1:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedPressureInput1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedPressureInput2:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedPressureInput2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedPressureInput3:
      {
        SBarometerMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedPressureInput3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInputFiltered1:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataImuInputFiltered1Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInputFiltered2:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataImuInputFiltered2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eImuInputFiltered3:
      {
        SImuMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataImuInputFiltered3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerData2:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedMagnetometerData2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerData3:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedMagnetometerData3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagneticHeading2:
      {
        SMagneticHeading oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataMagneticHeading2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eMagneticHeading3:
      {
        SMagneticHeading oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataMagneticHeading3Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame2:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedMagnetometerDataInVehicleFrame2Container.push_back(oPortData);
        }
      }
      break;
    
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame3:
      {
        SMagneticMeasurement oPortData;
    
        if (oPortData.FromByteVector(oBytes))
        {
          oPortDataCompensatedMagnetometerDataInVehicleFrame3Container.push_back(oPortData);
        }
      }
      break;

    default:
      BufferDataFromUserDefinedPorts(ePortId, oBytes);
      break;
    }

    urPositionInFile += uMessageLen;
  }
  else
  {
    ++urPositionInFile;
  }
}

void handleRunnableCallEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile)
{
  CSerialProtocol::SRunnableCallEventDebugMessage oMessage;
  bool bStatus = false;

  if ((urPositionInFile + sizeof(CSerialProtocol::SRunnableCallEventDebugMessage)) < uFileLength)
  {
    getRunnableCallEventMessage(upMessageStartAddress, oMessage);

    size_t uLengthInWords = (sizeof(oMessage) - sizeof(CSerialProtocol::CrcType_t)) / PROTOCOL_WORD_LEN;

    if (oMessage.uCrc_ == Crc32(reinterpret_cast<const uint32_t*>(upMessageStartAddress), uLengthInWords))
    {
      bStatus = true;
    }
    else if (0U == oMessage.uCrc_)
    {
      // In some cases, the CRC may be missing and represented as 0. We will process such messages, but print a warning.
      bStatus = true;
      std::cout << "Warning: CRC is missing for a runnable call event message. Position in file is " << urPositionInFile << std::endl;
    }
    else
    {
      std::cout << "CRC failed for a runnable call event message. Position in file is " << urPositionInFile << std::endl;
    }
  }

  if (true == bStatus)
  {
    CRte::ERunnableIds eRunnableId = static_cast<CRte::ERunnableIds>(oMessage.oRunnableCallData_.uRunnableId_);
    uint64_t uTimeStampUs = oMessage.oRunnableCallData_.uTimestampUs_;

    oRunnableCallEventDataContainer.push_back(oMessage.oRunnableCallData_);

    urPositionInFile += sizeof(oMessage);
  }
  else
  {
    ++urPositionInFile;
  }
}

void getWriteToPortEventPartialData(const uint8_t* upMessageStartAddress, CSerialProtocol::SWriteToPortEventPartialData& orPartialData)
{
  orPartialData = *reinterpret_cast<const CSerialProtocol::SWriteToPortEventPartialData*>(upMessageStartAddress + sizeof(CSerialProtocol::SMessageHeader));
}

void getRunnableCallEventMessage(const uint8_t* upMessageStartAddress, CSerialProtocol::SRunnableCallEventDebugMessage& orMessage)
{
  orMessage = *reinterpret_cast<const CSerialProtocol::SRunnableCallEventDebugMessage*>(upMessageStartAddress);
}

void CreateMatFile(string oName)
{
  TinyMATWriterFile* opMat = TinyMATWriter_open(oName.data());

  if (nullptr != opMat)
  {
    writePortImuInput1ToMat(opMat, oPortDataImuInput1Container);
    writePortPressureInput1ToMat(opMat, oPortDataPressureInput1Container);
    writePortMagnetometerInput1ToMat(opMat, oPortDataMagnetometerInput1Container);
    writePortMagnetometerInput2ToMat(opMat, oPortDataMagnetometerInput2Container);
    writePortMagnetometerInput3ToMat(opMat, oPortDataMagnetometerInput3Container);
    writePortPressureInput2ToMat(opMat, oPortDataPressureInput2Container);
    writePortPressureInput3ToMat(opMat, oPortDataPressureInput3Container);
    writePortImuInput2ToMat(opMat, oPortDataImuInput2Container);
    writePortImuInput3ToMat(opMat, oPortDataImuInput3Container);
    writePortSyncPulseTimeToMat(opMat, oPortDataSyncPulseTimeContainer);
    writePortCompensatedMagnetometerData1ToMat(opMat, oPortDataCompensatedMagnetometerData1Container);
    writePortMagneticHeading1ToMat(opMat, oPortDataMagneticHeading1Container);
    writePortCompensatedMagnetometerDataInVehicleFrame1ToMat(opMat, oPortDataCompensatedMagnetometerDataInVehicleFrame1Container);
    writePortSafePressureData1ToMat(opMat, oPortDataSafePressureData1Container);
    writePortSafePressureData2ToMat(opMat, oPortDataSafePressureData2Container);
    writePortSafePressureData3ToMat(opMat, oPortDataSafePressureData3Container);
    writePortVehicleAttitude1ToMat(opMat, oPortDataVehicleAttitude1Container);
    writePortVehicleAttitude2ToMat(opMat, oPortDataVehicleAttitude2Container);
    writePortVehicleAttitude3ToMat(opMat, oPortDataVehicleAttitude3Container);
    writePortCompensatedPressureInput1ToMat(opMat, oPortDataCompensatedPressureInput1Container);
    writePortCompensatedPressureInput2ToMat(opMat, oPortDataCompensatedPressureInput2Container);
    writePortCompensatedPressureInput3ToMat(opMat, oPortDataCompensatedPressureInput3Container);
    writePortImuInputFiltered1ToMat(opMat, oPortDataImuInputFiltered1Container);
    writePortImuInputFiltered2ToMat(opMat, oPortDataImuInputFiltered2Container);
    writePortImuInputFiltered3ToMat(opMat, oPortDataImuInputFiltered3Container);
    writePortCompensatedMagnetometerData2ToMat(opMat, oPortDataCompensatedMagnetometerData2Container);
    writePortCompensatedMagnetometerData3ToMat(opMat, oPortDataCompensatedMagnetometerData3Container);
    writePortMagneticHeading2ToMat(opMat, oPortDataMagneticHeading2Container);
    writePortMagneticHeading3ToMat(opMat, oPortDataMagneticHeading3Container);
    writePortCompensatedMagnetometerDataInVehicleFrame2ToMat(opMat, oPortDataCompensatedMagnetometerDataInVehicleFrame2Container);
    writePortCompensatedMagnetometerDataInVehicleFrame3ToMat(opMat, oPortDataCompensatedMagnetometerDataInVehicleFrame3Container);


    WriteUserDefinedPortsToMat(opMat);
    writeRunnableCallEventDataToMat(opMat, oRunnableCallEventDataContainer);

    TinyMATWriter_close(opMat);
  }
}

void writeRunnableCallEventDataToMat(TinyMATWriterFile* opMatWriter, std::vector<CSerialProtocol::SRunnableCallEventData>& orRunnableCallEventDataContainer)
{
  uint32_t* upRunnableId {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upAllPortsLocked { nullptr };

  try
  {
    upRunnableId = new uint32_t[orRunnableCallEventDataContainer.size()];
    upTimestampUs = new uint64_t[orRunnableCallEventDataContainer.size()];
    upAllPortsLocked = new uint8_t[orRunnableCallEventDataContainer.size()];

    for (uint64_t uIndex = 0; uIndex < orRunnableCallEventDataContainer.size(); uIndex++)
    {
      upRunnableId[uIndex] = orRunnableCallEventDataContainer[uIndex].uRunnableId_;
      upTimestampUs[uIndex] = orRunnableCallEventDataContainer[uIndex].uTimestampUs_;
      upAllPortsLocked[uIndex] = orRunnableCallEventDataContainer[uIndex].bAllPortsLocked_;
    }

    TinyMATWriter_startStruct(opMatWriter, "RunnableCallEventData");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uRunnableId_", upRunnableId, static_cast<int32_t>(orRunnableCallEventDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orRunnableCallEventDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "bAllPortsLocked_", upAllPortsLocked, static_cast<int32_t>(orRunnableCallEventDataContainer.size()));
    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct RunnableCallEventData..." << endl;
  }

  delete[] upRunnableId;
  delete[] upTimestampUs;
  delete[] upAllPortsLocked;
}


void writePortImuInput1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer)
{
  float* fpSpecificForceX {nullptr};
  float* fpSpecificForceY {nullptr};
  float* fpSpecificForceZ {nullptr};
  float* fpAngularRateX {nullptr};
  float* fpAngularRateY {nullptr};
  float* fpAngularRateZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upImuValid {nullptr};
  uint8_t* upTemperatureValid {nullptr};

  try
  {
    fpSpecificForceX = new float[orPortDataContainer.size()];
    fpSpecificForceY = new float[orPortDataContainer.size()];
    fpSpecificForceZ = new float[orPortDataContainer.size()];
    fpAngularRateX = new float[orPortDataContainer.size()];
    fpAngularRateY = new float[orPortDataContainer.size()];
    fpAngularRateZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upImuValid = new uint8_t[orPortDataContainer.size()];
    upTemperatureValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpSpecificForceX[uIndex] = orPortDataContainer[uIndex].fSpecificForceX_;
      fpSpecificForceY[uIndex] = orPortDataContainer[uIndex].fSpecificForceY_;
      fpSpecificForceZ[uIndex] = orPortDataContainer[uIndex].fSpecificForceZ_;
      fpAngularRateX[uIndex] = orPortDataContainer[uIndex].fAngularRateX_;
      fpAngularRateY[uIndex] = orPortDataContainer[uIndex].fAngularRateY_;
      fpAngularRateZ[uIndex] = orPortDataContainer[uIndex].fAngularRateZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upImuValid[uIndex] = orPortDataContainer[uIndex].uImuValid_;
      upTemperatureValid[uIndex] = orPortDataContainer[uIndex].uTemperatureValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortImuInput1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceX_", fpSpecificForceX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceY_", fpSpecificForceY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceZ_", fpSpecificForceZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateX_", fpAngularRateX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateY_", fpAngularRateY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateZ_", fpAngularRateZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uImuValid_", upImuValid, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTemperatureValid_", upTemperatureValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortImuInput1..." << endl;
  }

  delete [] fpSpecificForceX;
  delete [] fpSpecificForceY;
  delete [] fpSpecificForceZ;
  delete [] fpAngularRateX;
  delete [] fpAngularRateY;
  delete [] fpAngularRateZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upImuValid;
  delete [] upTemperatureValid;

}

void writePortPressureInput1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortPressureInput1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortPressureInput1..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortMagnetometerInput1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortMagnetometerInput1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortMagnetometerInput1..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortMagnetometerInput2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortMagnetometerInput2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortMagnetometerInput2..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortMagnetometerInput3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortMagnetometerInput3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortMagnetometerInput3..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortPressureInput2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortPressureInput2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortPressureInput2..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortPressureInput3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortPressureInput3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortPressureInput3..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortImuInput2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer)
{
  float* fpSpecificForceX {nullptr};
  float* fpSpecificForceY {nullptr};
  float* fpSpecificForceZ {nullptr};
  float* fpAngularRateX {nullptr};
  float* fpAngularRateY {nullptr};
  float* fpAngularRateZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upImuValid {nullptr};
  uint8_t* upTemperatureValid {nullptr};

  try
  {
    fpSpecificForceX = new float[orPortDataContainer.size()];
    fpSpecificForceY = new float[orPortDataContainer.size()];
    fpSpecificForceZ = new float[orPortDataContainer.size()];
    fpAngularRateX = new float[orPortDataContainer.size()];
    fpAngularRateY = new float[orPortDataContainer.size()];
    fpAngularRateZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upImuValid = new uint8_t[orPortDataContainer.size()];
    upTemperatureValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpSpecificForceX[uIndex] = orPortDataContainer[uIndex].fSpecificForceX_;
      fpSpecificForceY[uIndex] = orPortDataContainer[uIndex].fSpecificForceY_;
      fpSpecificForceZ[uIndex] = orPortDataContainer[uIndex].fSpecificForceZ_;
      fpAngularRateX[uIndex] = orPortDataContainer[uIndex].fAngularRateX_;
      fpAngularRateY[uIndex] = orPortDataContainer[uIndex].fAngularRateY_;
      fpAngularRateZ[uIndex] = orPortDataContainer[uIndex].fAngularRateZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upImuValid[uIndex] = orPortDataContainer[uIndex].uImuValid_;
      upTemperatureValid[uIndex] = orPortDataContainer[uIndex].uTemperatureValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortImuInput2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceX_", fpSpecificForceX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceY_", fpSpecificForceY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceZ_", fpSpecificForceZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateX_", fpAngularRateX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateY_", fpAngularRateY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateZ_", fpAngularRateZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uImuValid_", upImuValid, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTemperatureValid_", upTemperatureValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortImuInput2..." << endl;
  }

  delete [] fpSpecificForceX;
  delete [] fpSpecificForceY;
  delete [] fpSpecificForceZ;
  delete [] fpAngularRateX;
  delete [] fpAngularRateY;
  delete [] fpAngularRateZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upImuValid;
  delete [] upTemperatureValid;

}

void writePortImuInput3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer)
{
  float* fpSpecificForceX {nullptr};
  float* fpSpecificForceY {nullptr};
  float* fpSpecificForceZ {nullptr};
  float* fpAngularRateX {nullptr};
  float* fpAngularRateY {nullptr};
  float* fpAngularRateZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upImuValid {nullptr};
  uint8_t* upTemperatureValid {nullptr};

  try
  {
    fpSpecificForceX = new float[orPortDataContainer.size()];
    fpSpecificForceY = new float[orPortDataContainer.size()];
    fpSpecificForceZ = new float[orPortDataContainer.size()];
    fpAngularRateX = new float[orPortDataContainer.size()];
    fpAngularRateY = new float[orPortDataContainer.size()];
    fpAngularRateZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upImuValid = new uint8_t[orPortDataContainer.size()];
    upTemperatureValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpSpecificForceX[uIndex] = orPortDataContainer[uIndex].fSpecificForceX_;
      fpSpecificForceY[uIndex] = orPortDataContainer[uIndex].fSpecificForceY_;
      fpSpecificForceZ[uIndex] = orPortDataContainer[uIndex].fSpecificForceZ_;
      fpAngularRateX[uIndex] = orPortDataContainer[uIndex].fAngularRateX_;
      fpAngularRateY[uIndex] = orPortDataContainer[uIndex].fAngularRateY_;
      fpAngularRateZ[uIndex] = orPortDataContainer[uIndex].fAngularRateZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upImuValid[uIndex] = orPortDataContainer[uIndex].uImuValid_;
      upTemperatureValid[uIndex] = orPortDataContainer[uIndex].uTemperatureValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortImuInput3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceX_", fpSpecificForceX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceY_", fpSpecificForceY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceZ_", fpSpecificForceZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateX_", fpAngularRateX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateY_", fpAngularRateY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateZ_", fpAngularRateZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uImuValid_", upImuValid, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTemperatureValid_", upTemperatureValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortImuInput3..." << endl;
  }

  delete [] fpSpecificForceX;
  delete [] fpSpecificForceY;
  delete [] fpSpecificForceZ;
  delete [] fpAngularRateX;
  delete [] fpAngularRateY;
  delete [] fpAngularRateZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upImuValid;
  delete [] upTemperatureValid;

}

void writePortSyncPulseTimeToMat(TinyMATWriterFile* opMatWriter, std::vector<STimeOfSyncPulse>& orPortDataContainer)
{
  uint64_t* upPulseTimeUs {nullptr};

  try
  {
    upPulseTimeUs = new uint64_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      upPulseTimeUs[uIndex] = orPortDataContainer[uIndex].uPulseTimeUs_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortSyncPulseTime");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uPulseTimeUs_", upPulseTimeUs, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortSyncPulseTime..." << endl;
  }

  delete [] upPulseTimeUs;

}

void writePortCompensatedMagnetometerData1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedMagnetometerData1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedMagnetometerData1..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortMagneticHeading1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticHeading>& orPortDataContainer)
{
  float* fpMagneticHeading {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpMagneticHeading = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpMagneticHeading[uIndex] = orPortDataContainer[uIndex].fMagneticHeading_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortMagneticHeading1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fMagneticHeading_", fpMagneticHeading, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortMagneticHeading1..." << endl;
  }

  delete [] fpMagneticHeading;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortCompensatedMagnetometerDataInVehicleFrame1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedMagnetometerDataInVehicleFrame1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedMagnetometerDataInVehicleFrame1..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortSafePressureData1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortSafePressureData1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortSafePressureData1..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortSafePressureData2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortSafePressureData2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortSafePressureData2..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortSafePressureData3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortSafePressureData3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortSafePressureData3..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortVehicleAttitude1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SAttitudeData>& orPortDataContainer)
{
  float* fpRoll {nullptr};
  float* fpPitch {nullptr};
  float* fpAttitudeStd1 {nullptr};
  float* fpAttitudeStd2 {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpRoll = new float[orPortDataContainer.size()];
    fpPitch = new float[orPortDataContainer.size()];
    fpAttitudeStd1 = new float[orPortDataContainer.size()];
    fpAttitudeStd2 = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpRoll[uIndex] = orPortDataContainer[uIndex].fRoll_;
      fpPitch[uIndex] = orPortDataContainer[uIndex].fPitch_;
      fpAttitudeStd1[uIndex] = orPortDataContainer[uIndex].fAttitudeStd1_;
      fpAttitudeStd2[uIndex] = orPortDataContainer[uIndex].fAttitudeStd2_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortVehicleAttitude1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fRoll_", fpRoll, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPitch_", fpPitch, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAttitudeStd1_", fpAttitudeStd1, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAttitudeStd2_", fpAttitudeStd2, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortVehicleAttitude1..." << endl;
  }

  delete [] fpRoll;
  delete [] fpPitch;
  delete [] fpAttitudeStd1;
  delete [] fpAttitudeStd2;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortVehicleAttitude2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SAttitudeData>& orPortDataContainer)
{
  float* fpRoll {nullptr};
  float* fpPitch {nullptr};
  float* fpAttitudeStd1 {nullptr};
  float* fpAttitudeStd2 {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpRoll = new float[orPortDataContainer.size()];
    fpPitch = new float[orPortDataContainer.size()];
    fpAttitudeStd1 = new float[orPortDataContainer.size()];
    fpAttitudeStd2 = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpRoll[uIndex] = orPortDataContainer[uIndex].fRoll_;
      fpPitch[uIndex] = orPortDataContainer[uIndex].fPitch_;
      fpAttitudeStd1[uIndex] = orPortDataContainer[uIndex].fAttitudeStd1_;
      fpAttitudeStd2[uIndex] = orPortDataContainer[uIndex].fAttitudeStd2_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortVehicleAttitude2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fRoll_", fpRoll, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPitch_", fpPitch, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAttitudeStd1_", fpAttitudeStd1, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAttitudeStd2_", fpAttitudeStd2, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortVehicleAttitude2..." << endl;
  }

  delete [] fpRoll;
  delete [] fpPitch;
  delete [] fpAttitudeStd1;
  delete [] fpAttitudeStd2;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortVehicleAttitude3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SAttitudeData>& orPortDataContainer)
{
  float* fpRoll {nullptr};
  float* fpPitch {nullptr};
  float* fpAttitudeStd1 {nullptr};
  float* fpAttitudeStd2 {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpRoll = new float[orPortDataContainer.size()];
    fpPitch = new float[orPortDataContainer.size()];
    fpAttitudeStd1 = new float[orPortDataContainer.size()];
    fpAttitudeStd2 = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpRoll[uIndex] = orPortDataContainer[uIndex].fRoll_;
      fpPitch[uIndex] = orPortDataContainer[uIndex].fPitch_;
      fpAttitudeStd1[uIndex] = orPortDataContainer[uIndex].fAttitudeStd1_;
      fpAttitudeStd2[uIndex] = orPortDataContainer[uIndex].fAttitudeStd2_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortVehicleAttitude3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fRoll_", fpRoll, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPitch_", fpPitch, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAttitudeStd1_", fpAttitudeStd1, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAttitudeStd2_", fpAttitudeStd2, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortVehicleAttitude3..." << endl;
  }

  delete [] fpRoll;
  delete [] fpPitch;
  delete [] fpAttitudeStd1;
  delete [] fpAttitudeStd2;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortCompensatedPressureInput1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedPressureInput1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedPressureInput1..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortCompensatedPressureInput2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedPressureInput2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedPressureInput2..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortCompensatedPressureInput3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SBarometerMeasurement>& orPortDataContainer)
{
  float* fpPressure {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpPressure = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpPressure[uIndex] = orPortDataContainer[uIndex].fPressure_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedPressureInput3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fPressure_", fpPressure, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedPressureInput3..." << endl;
  }

  delete [] fpPressure;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortImuInputFiltered1ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer)
{
  float* fpSpecificForceX {nullptr};
  float* fpSpecificForceY {nullptr};
  float* fpSpecificForceZ {nullptr};
  float* fpAngularRateX {nullptr};
  float* fpAngularRateY {nullptr};
  float* fpAngularRateZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upImuValid {nullptr};
  uint8_t* upTemperatureValid {nullptr};

  try
  {
    fpSpecificForceX = new float[orPortDataContainer.size()];
    fpSpecificForceY = new float[orPortDataContainer.size()];
    fpSpecificForceZ = new float[orPortDataContainer.size()];
    fpAngularRateX = new float[orPortDataContainer.size()];
    fpAngularRateY = new float[orPortDataContainer.size()];
    fpAngularRateZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upImuValid = new uint8_t[orPortDataContainer.size()];
    upTemperatureValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpSpecificForceX[uIndex] = orPortDataContainer[uIndex].fSpecificForceX_;
      fpSpecificForceY[uIndex] = orPortDataContainer[uIndex].fSpecificForceY_;
      fpSpecificForceZ[uIndex] = orPortDataContainer[uIndex].fSpecificForceZ_;
      fpAngularRateX[uIndex] = orPortDataContainer[uIndex].fAngularRateX_;
      fpAngularRateY[uIndex] = orPortDataContainer[uIndex].fAngularRateY_;
      fpAngularRateZ[uIndex] = orPortDataContainer[uIndex].fAngularRateZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upImuValid[uIndex] = orPortDataContainer[uIndex].uImuValid_;
      upTemperatureValid[uIndex] = orPortDataContainer[uIndex].uTemperatureValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortImuInputFiltered1");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceX_", fpSpecificForceX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceY_", fpSpecificForceY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceZ_", fpSpecificForceZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateX_", fpAngularRateX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateY_", fpAngularRateY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateZ_", fpAngularRateZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uImuValid_", upImuValid, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTemperatureValid_", upTemperatureValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortImuInputFiltered1..." << endl;
  }

  delete [] fpSpecificForceX;
  delete [] fpSpecificForceY;
  delete [] fpSpecificForceZ;
  delete [] fpAngularRateX;
  delete [] fpAngularRateY;
  delete [] fpAngularRateZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upImuValid;
  delete [] upTemperatureValid;

}

void writePortImuInputFiltered2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer)
{
  float* fpSpecificForceX {nullptr};
  float* fpSpecificForceY {nullptr};
  float* fpSpecificForceZ {nullptr};
  float* fpAngularRateX {nullptr};
  float* fpAngularRateY {nullptr};
  float* fpAngularRateZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upImuValid {nullptr};
  uint8_t* upTemperatureValid {nullptr};

  try
  {
    fpSpecificForceX = new float[orPortDataContainer.size()];
    fpSpecificForceY = new float[orPortDataContainer.size()];
    fpSpecificForceZ = new float[orPortDataContainer.size()];
    fpAngularRateX = new float[orPortDataContainer.size()];
    fpAngularRateY = new float[orPortDataContainer.size()];
    fpAngularRateZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upImuValid = new uint8_t[orPortDataContainer.size()];
    upTemperatureValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpSpecificForceX[uIndex] = orPortDataContainer[uIndex].fSpecificForceX_;
      fpSpecificForceY[uIndex] = orPortDataContainer[uIndex].fSpecificForceY_;
      fpSpecificForceZ[uIndex] = orPortDataContainer[uIndex].fSpecificForceZ_;
      fpAngularRateX[uIndex] = orPortDataContainer[uIndex].fAngularRateX_;
      fpAngularRateY[uIndex] = orPortDataContainer[uIndex].fAngularRateY_;
      fpAngularRateZ[uIndex] = orPortDataContainer[uIndex].fAngularRateZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upImuValid[uIndex] = orPortDataContainer[uIndex].uImuValid_;
      upTemperatureValid[uIndex] = orPortDataContainer[uIndex].uTemperatureValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortImuInputFiltered2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceX_", fpSpecificForceX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceY_", fpSpecificForceY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceZ_", fpSpecificForceZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateX_", fpAngularRateX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateY_", fpAngularRateY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateZ_", fpAngularRateZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uImuValid_", upImuValid, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTemperatureValid_", upTemperatureValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortImuInputFiltered2..." << endl;
  }

  delete [] fpSpecificForceX;
  delete [] fpSpecificForceY;
  delete [] fpSpecificForceZ;
  delete [] fpAngularRateX;
  delete [] fpAngularRateY;
  delete [] fpAngularRateZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upImuValid;
  delete [] upTemperatureValid;

}

void writePortImuInputFiltered3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SImuMeasurement>& orPortDataContainer)
{
  float* fpSpecificForceX {nullptr};
  float* fpSpecificForceY {nullptr};
  float* fpSpecificForceZ {nullptr};
  float* fpAngularRateX {nullptr};
  float* fpAngularRateY {nullptr};
  float* fpAngularRateZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upImuValid {nullptr};
  uint8_t* upTemperatureValid {nullptr};

  try
  {
    fpSpecificForceX = new float[orPortDataContainer.size()];
    fpSpecificForceY = new float[orPortDataContainer.size()];
    fpSpecificForceZ = new float[orPortDataContainer.size()];
    fpAngularRateX = new float[orPortDataContainer.size()];
    fpAngularRateY = new float[orPortDataContainer.size()];
    fpAngularRateZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upImuValid = new uint8_t[orPortDataContainer.size()];
    upTemperatureValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpSpecificForceX[uIndex] = orPortDataContainer[uIndex].fSpecificForceX_;
      fpSpecificForceY[uIndex] = orPortDataContainer[uIndex].fSpecificForceY_;
      fpSpecificForceZ[uIndex] = orPortDataContainer[uIndex].fSpecificForceZ_;
      fpAngularRateX[uIndex] = orPortDataContainer[uIndex].fAngularRateX_;
      fpAngularRateY[uIndex] = orPortDataContainer[uIndex].fAngularRateY_;
      fpAngularRateZ[uIndex] = orPortDataContainer[uIndex].fAngularRateZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upImuValid[uIndex] = orPortDataContainer[uIndex].uImuValid_;
      upTemperatureValid[uIndex] = orPortDataContainer[uIndex].uTemperatureValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortImuInputFiltered3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceX_", fpSpecificForceX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceY_", fpSpecificForceY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fSpecificForceZ_", fpSpecificForceZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateX_", fpAngularRateX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateY_", fpAngularRateY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fAngularRateZ_", fpAngularRateZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uImuValid_", upImuValid, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTemperatureValid_", upTemperatureValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortImuInputFiltered3..." << endl;
  }

  delete [] fpSpecificForceX;
  delete [] fpSpecificForceY;
  delete [] fpSpecificForceZ;
  delete [] fpAngularRateX;
  delete [] fpAngularRateY;
  delete [] fpAngularRateZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upImuValid;
  delete [] upTemperatureValid;

}

void writePortCompensatedMagnetometerData2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedMagnetometerData2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedMagnetometerData2..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortCompensatedMagnetometerData3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedMagnetometerData3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedMagnetometerData3..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortMagneticHeading2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticHeading>& orPortDataContainer)
{
  float* fpMagneticHeading {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpMagneticHeading = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpMagneticHeading[uIndex] = orPortDataContainer[uIndex].fMagneticHeading_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortMagneticHeading2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fMagneticHeading_", fpMagneticHeading, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortMagneticHeading2..." << endl;
  }

  delete [] fpMagneticHeading;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortMagneticHeading3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticHeading>& orPortDataContainer)
{
  float* fpMagneticHeading {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpMagneticHeading = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpMagneticHeading[uIndex] = orPortDataContainer[uIndex].fMagneticHeading_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortMagneticHeading3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fMagneticHeading_", fpMagneticHeading, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortMagneticHeading3..." << endl;
  }

  delete [] fpMagneticHeading;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortCompensatedMagnetometerDataInVehicleFrame2ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedMagnetometerDataInVehicleFrame2");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedMagnetometerDataInVehicleFrame2..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

void writePortCompensatedMagnetometerDataInVehicleFrame3ToMat(TinyMATWriterFile* opMatWriter, std::vector<SMagneticMeasurement>& orPortDataContainer)
{
  float* fpVectorX {nullptr};
  float* fpVectorY {nullptr};
  float* fpVectorZ {nullptr};
  float* fpTemperature {nullptr};
  uint64_t* upTimestampUs {nullptr};
  uint8_t* upValid {nullptr};

  try
  {
    fpVectorX = new float[orPortDataContainer.size()];
    fpVectorY = new float[orPortDataContainer.size()];
    fpVectorZ = new float[orPortDataContainer.size()];
    fpTemperature = new float[orPortDataContainer.size()];
    upTimestampUs = new uint64_t[orPortDataContainer.size()];
    upValid = new uint8_t[orPortDataContainer.size()];


    for (uint64_t uIndex = 0; uIndex < orPortDataContainer.size(); uIndex++)
    {
      fpVectorX[uIndex] = orPortDataContainer[uIndex].fVectorX_;
      fpVectorY[uIndex] = orPortDataContainer[uIndex].fVectorY_;
      fpVectorZ[uIndex] = orPortDataContainer[uIndex].fVectorZ_;
      fpTemperature[uIndex] = orPortDataContainer[uIndex].fTemperature_;
      upTimestampUs[uIndex] = orPortDataContainer[uIndex].uTimestampUs_;
      upValid[uIndex] = orPortDataContainer[uIndex].uValid_;

    }

    TinyMATWriter_startStruct(opMatWriter, "PortCompensatedMagnetometerDataInVehicleFrame3");
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorX_", fpVectorX, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorY_", fpVectorY, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fVectorZ_", fpVectorZ, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "fTemperature_", fpTemperature, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uTimestampUs_", upTimestampUs, static_cast<int32_t>(orPortDataContainer.size()));
    TinyMATWriter_writeVectorAsRow(opMatWriter, "uValid_", upValid, static_cast<int32_t>(orPortDataContainer.size()));

    TinyMATWriter_endStruct(opMatWriter);
  }
  catch (std::bad_alloc)
  {
    cout << "Failed to allocate memory." << "Skipping the struct PortCompensatedMagnetometerDataInVehicleFrame3..." << endl;
  }

  delete [] fpVectorX;
  delete [] fpVectorY;
  delete [] fpVectorZ;
  delete [] fpTemperature;
  delete [] upTimestampUs;
  delete [] upValid;

}

