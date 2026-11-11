/**
 * @file DebugOutputToMat.cpp
 * @brief Converter of binary debug data to MAT.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date {DATE_GENERATED}
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

{DECLARATION_OF_PORT_DATA_VECTORS}


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

{DECLARATIONS_OF_MAT_WRITERS}

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
{PORT_ID_SWITCH}
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
{WRITE_PORT_DATA_TO_MAT}

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

{IMPLEMENTATIONS_OF_PORT_DATA_WRITERS}
