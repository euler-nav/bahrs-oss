/// @file BahrsReplayTool.cpp
/// @brief Implementation of a tool to replay BAHRS output data on a PC and analyze the behavior of SW components.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include <iostream>
#include <math.h>
#include <fstream>
#include <vector>
#include "CSerialProtocol.h"
#include "CRte.h"
#include "CPressureCompensator.h"
#include "Crc32.h"
#include "WriteToRtePort.h"
#ifdef SEND_DEBUG_OUTPUT
#include "CFileDebugOutputHandler.h"
#endif // SEND_DEBUG_OUTPUT

using namespace std;


/// @brief Extract partial data of the write to port event from a byte array loaded to RAM.
/// @param upMessageStartAddress Pointer to the first byte of the debug message.
/// @param orPartialData Reference to a structure to be filled.
static void getWriteToPortEventPartialData(const uint8_t* upMessageStartAddress, CSerialProtocol::SWriteToPortEventPartialData& orPartialData);

/// @brief Extract a runnable call debug message from a byte array loaded to RAM.
/// @param upMessageStartAddress Pointer to the first byte of the debug message.
/// @param orMessage Reference to a structure to be filled.
static void getRunnableCallEventMessage(const uint8_t* upMessageStartAddress, CSerialProtocol::SRunnableCallEventDebugMessage& orMessage);

/// @brief Handle the write to port event.
/// Extracts debug data from a byte array, checks CRC, and writes data to a port.
/// @param upMessageStartAddress Pointer to the first byte of the debug message.
/// @param uFileLength Length of the log file.
/// @param urPositionInFile Current position in the file.
static void handlePortWriteEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile);

/// @brief Handle runnable call event.
/// Extracts debug data from a byte array, checks CRC, and calls a runnable.
/// @param upMessageStartAddress Pointer to the first byte of the debug message.
/// @param uFileLength Length of the log file.
/// @param urPositionInFile Current position in the file.
static void handleRunnableCallEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile);

static void SetPortsToRecordInReplayTool();
static bool IsRecomputedPort(CRte::EPortIds ePortId);

static uint32_t uDebugMessageCount = 0U;
static std::optional<uint64_t> uLastCyclicTaskTimestampUs{};
static uint64_t uTaskCounter5ms{ 0U };
static constexpr uint64_t skuTaskPeriod5msUs{ 5'000U };
uint64_t uReplayTimestampUs { 0U };

int main(int argc, char* argv[])
{
  ifstream oInputFile;
  uint8_t* upBuff;
  size_t uFileLength = 0U;
  string oFilename;
  string oOutputFilenameDebug;
  
  if (argc == 3)
  {
    oFilename.append(argv[1]);
    oOutputFilenameDebug.append(argv[2]);
  }
  else
  {
    cout << "Wrong command line parameter count. Please enter input file name. The program will terminate." << endl;
    cout << "Usage: <input.bin> <debug_output.bin>" << endl;
    return 1;
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
    cout << "Failed to open the input file " << oFilename << ". The program will terminate." << endl;
    return 1;
  }

#ifdef SEND_DEBUG_OUTPUT
  if (false == CFileDebugOutputHandler::GetInstance().Init(oOutputFilenameDebug))
  {
    cout << "Failed to open the debug output file " << oOutputFilenameDebug << ". The program will terminate." << endl;
    return 1;
  }
#endif // SEND_DEBUG_OUTPUT

  CRte::GetInstance().Init();

  for (uint32_t uIndex = 0U; uIndex < CPressureCompensator::skuInstanceCount_; ++uIndex)
  {
    CPressureCompensator::GetInstance(uIndex).Init();
  }

  for (uint32_t uIndex = 0U; uIndex < CImuPreFilterSwc::skuInstanceCount_; ++uIndex)
  {
    CImuPreFilterSwc::GetInstance(uIndex).Init();
  }

  for (uint32_t uIndex = 0U; uIndex < CMagnetometerCompensator::skuInstanceCount_; ++uIndex)
  {
    CMagnetometerCompensator::GetInstance(uIndex).Init();
  }

  COutputTransformer::GetInstance().Init();
  SetPortsToRecordInReplayTool();

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

  delete [] upBuff;

  cout << "Processed " << uDebugMessageCount << " debug messages." << endl;

  return 0;
}

static void getWriteToPortEventPartialData(const uint8_t* upMessageStartAddress, CSerialProtocol::SWriteToPortEventPartialData& orPartialData)
{
  orPartialData = *reinterpret_cast<const CSerialProtocol::SWriteToPortEventPartialData*>(upMessageStartAddress + sizeof(CSerialProtocol::SMessageHeader));
}

static void getRunnableCallEventMessage(const uint8_t* upMessageStartAddress, CSerialProtocol::SRunnableCallEventDebugMessage& orMessage)
{
  orMessage = *reinterpret_cast<const CSerialProtocol::SRunnableCallEventDebugMessage*>(upMessageStartAddress);
}

static void handlePortWriteEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile)
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
        ++uDebugMessageCount;
        bStatus = true;
      }
    }
  }


  if (true == bStatus)
  {
    CRte::EPortIds ePortId = static_cast<CRte::EPortIds>(oEventPartialData.uPortId_);

    if (false == IsRecomputedPort(ePortId))
    {
      unsigned uDataOffset = sizeof(CSerialProtocol::SMessageHeader) + sizeof(CSerialProtocol::SWriteToPortEventPartialData);
      std::vector<uint8_t> oBytes(upMessageStartAddress + uDataOffset, upMessageStartAddress + uDataOffset + oEventPartialData.uDataLen_);

      uReplayTimestampUs = oEventPartialData.uTimestampUs_;
      WriteToRtePort(ePortId, oBytes);

      switch (ePortId)
      {
        case CRte::EPortIds::ePressureInput1:
          CPressureCompensator::GetInstance(0U).CompensateMeasurements();
          break;
        case CRte::EPortIds::ePressureInput2:
          CPressureCompensator::GetInstance(1U).CompensateMeasurements();
          break;
        case CRte::EPortIds::ePressureInput3:
          CPressureCompensator::GetInstance(2U).CompensateMeasurements();
          break;
        case CRte::EPortIds::eImuInput1:
          CImuPreFilterSwc::GetInstance(0U).Run();
          break;
        case CRte::EPortIds::eImuInput2:
          CImuPreFilterSwc::GetInstance(1U).Run();
          break;
        case CRte::EPortIds::eImuInput3:
          CImuPreFilterSwc::GetInstance(2U).Run();
          break;
        case CRte::EPortIds::eMagnetometerInput1:
          CMagnetometerCompensator::GetInstance(0U).CompensateMeasurements();
          COutputTransformer::GetInstance().TransformMagnetometerInput(0U);
          break;
        case CRte::EPortIds::eMagnetometerInput2:
          CMagnetometerCompensator::GetInstance(1U).CompensateMeasurements();
          COutputTransformer::GetInstance().TransformMagnetometerInput(1U);
          break;
        case CRte::EPortIds::eMagnetometerInput3:
          CMagnetometerCompensator::GetInstance(2U).CompensateMeasurements();
          COutputTransformer::GetInstance().TransformMagnetometerInput(2U);
          break;
        default:
          break;
      }
    }

    urPositionInFile += uMessageLen;
  }
  else
  {
    ++urPositionInFile;
  }
}

static void handleRunnableCallEvent(const uint8_t* upMessageStartAddress, size_t uFileLength, size_t& urPositionInFile)
{
  CSerialProtocol::SRunnableCallEventDebugMessage oMessage;
  bool bStatus = false;

  if ((urPositionInFile + sizeof(CSerialProtocol::SRunnableCallEventDebugMessage)) < uFileLength)
  {
    getRunnableCallEventMessage(upMessageStartAddress, oMessage);

    size_t uLengthInWords = (sizeof(oMessage) - sizeof(CSerialProtocol::CrcType_t)) / PROTOCOL_WORD_LEN;

    if (oMessage.uCrc_ == Crc32(reinterpret_cast<const uint32_t*>(upMessageStartAddress), uLengthInWords))
    {
      ++uDebugMessageCount;
      bStatus = true;
    }
  }

  if (true == bStatus)
  {
    CRte::ERunnableIds eRunnableId = static_cast<CRte::ERunnableIds>(oMessage.oRunnableCallData_.uRunnableId_);
    uint64_t uTimeStampUs = oMessage.oRunnableCallData_.uTimestampUs_;

    if (false == uLastCyclicTaskTimestampUs.has_value())
    {
      uLastCyclicTaskTimestampUs = uTimeStampUs;
    }

    if (uTimeStampUs < uLastCyclicTaskTimestampUs.value())
    {
      cout << "Warning: Timestamps of runnable call events are not in ascending order. This may lead to inaccurate replay results." << endl;
    }
    else
    {
      while ((uLastCyclicTaskTimestampUs.value() += skuTaskPeriod5msUs) < uTimeStampUs)
      {
        uReplayTimestampUs = uLastCyclicTaskTimestampUs.value();
        // Do 5ms tasks
        COutputTransformer::GetInstance().TransformImuSignals();

        if ((uTaskCounter5ms % 2U) == 0U)
        {
          // Do 10ms tasks
          uReplayTimestampUs = uLastCyclicTaskTimestampUs.value();
          COutputTransformer::GetInstance().TransformOrientation();
          CAttitudeMonitorSwc::GetInstance().Run();
          CVerticalChannelMonitorSwc::GetInstance().Run();

          for (uint32_t uFilterIndex = 0U; uFilterIndex < CMagneticHeadingFilterSwc::skuInstanceCount_; ++uFilterIndex)
          {
            CMagneticHeadingFilterSwc::GetInstance(uFilterIndex).SetAttitudeInput();
          }

          for (uint32_t uFilterIndex = 0U; uFilterIndex < CMagneticHeadingFilterSwc::skuInstanceCount_; ++uFilterIndex)
          {
            CMagneticHeadingFilterSwc::GetInstance(uFilterIndex).Step(uLastCyclicTaskTimestampUs.value());
          }

          CMagneticHeadingMonitorSwc::GetInstance().Run();
        }

        ++uTaskCounter5ms;
      }

      uLastCyclicTaskTimestampUs.value() -= skuTaskPeriod5msUs; // Adjust back on overshoot
    }

    uReplayTimestampUs = uTimeStampUs;

    switch (eRunnableId)
    {
      case CRte::ERunnableIds::eRunnableBahrsFilterStep1:
      {
        CBahrsFilterSwc::GetInstance(0).Step(uTimeStampUs);
        break;
      }
      case CRte::ERunnableIds::eRunnableBahrsFilterStep2:
      {
        CBahrsFilterSwc::GetInstance(1).Step(uTimeStampUs);
        break;
      }
      case CRte::ERunnableIds::eRunnableBahrsFilterStep3:
      {
        CBahrsFilterSwc::GetInstance(2).Step(uTimeStampUs);
        break;
      }
      case CRte::ERunnableIds::eRunnableBahrsFilterSetImuInput1:
      {
        CBahrsFilterSwc::GetInstance(0).SetImuInput();
        break;
      }
      case CRte::ERunnableIds::eRunnableBahrsFilterSetImuInput2:
      {
        CBahrsFilterSwc::GetInstance(1).SetImuInput();
        break;
      }
      case CRte::ERunnableIds::eRunnableBahrsFilterSetImuInput3:
      {
        CBahrsFilterSwc::GetInstance(2).SetImuInput();
        break;
      }
      case CRte::ERunnableIds::eRunnableBahrsFilterSetPressureInput1:
      {
        CBahrsFilterSwc::GetInstance(0).SetPressureInput();
        break;
      }
      case CRte::ERunnableIds::eRunnableBahrsFilterSetPressureInput2:
      {
        CBahrsFilterSwc::GetInstance(1).SetPressureInput();
        break;
      }
      case CRte::ERunnableIds::eRunnableBahrsFilterSetPressureInput3:
      {
        CBahrsFilterSwc::GetInstance(2).SetPressureInput();
        break;
      }
      case CRte::ERunnableIds::eRunnableImuMonitorRun:
      {
        CImuMonitorSwc::GetInstance().Run();
        break;
      }
      case CRte::ERunnableIds::eRunnableBarometerMonitorRun:
      {
        CBaroMonitorSwc::GetInstance().Run();
        break;
      }      
      default:
      {
        // No action needed for other runnables
        break;
      }
    }

    urPositionInFile += sizeof(oMessage);
  }
  else
  {
    ++urPositionInFile;
  }
}

void SetPortsToRecordInReplayTool()
{
  CRte::GetInstance().oPortImuInputFiltered1_.SetRecordForDebug(true);
  CRte::GetInstance().oPortImuInputFiltered2_.SetRecordForDebug(true);
  CRte::GetInstance().oPortImuInputFiltered3_.SetRecordForDebug(true);

  CRte::GetInstance().oPortCompensatedPressureInput1_.SetRecordForDebug(true);
  CRte::GetInstance().oPortCompensatedPressureInput2_.SetRecordForDebug(true);
  CRte::GetInstance().oPortCompensatedPressureInput3_.SetRecordForDebug(true);

  CRte::GetInstance().oPortBahrsFilterOutput1_.SetRecordForDebug(true);
  CRte::GetInstance().oPortBahrsFilterOutput2_.SetRecordForDebug(true);
  CRte::GetInstance().oPortBahrsFilterOutput3_.SetRecordForDebug(true);

  CRte::GetInstance().oPortImuDataAfterMonitor_.SetRecordForDebug(true);
  CRte::GetInstance().oPortImuOutput_.SetRecordForDebug(true);

  CRte::GetInstance().oPortSafePressureData1_.SetRecordForDebug(true);
  CRte::GetInstance().oPortSafePressureData2_.SetRecordForDebug(true);
  CRte::GetInstance().oPortSafePressureData3_.SetRecordForDebug(true);

  CRte::GetInstance().oPortVehicleAttitude1_.SetRecordForDebug(true);
  CRte::GetInstance().oPortVehicleAttitude2_.SetRecordForDebug(true);
  CRte::GetInstance().oPortVehicleAttitude3_.SetRecordForDebug(true);

  CRte::GetInstance().oPortSafeVehicleAttitude_.SetRecordForDebug(true);
  CRte::GetInstance().oPortSafeVerticalChannelData_.SetRecordForDebug(true);
  
  CRte::GetInstance().oPortCompensatedMagnetometerData1_.SetRecordForDebug(true);
  CRte::GetInstance().oPortCompensatedMagnetometerData2_.SetRecordForDebug(true);
  CRte::GetInstance().oPortCompensatedMagnetometerData3_.SetRecordForDebug(true);
  CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame1_.SetRecordForDebug(true);
  CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame2_.SetRecordForDebug(true);
  CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame3_.SetRecordForDebug(true);
  CRte::GetInstance().oPortMagneticHeading1_.SetRecordForDebug(true);
  CRte::GetInstance().oPortMagneticHeading2_.SetRecordForDebug(true);
  CRte::GetInstance().oPortMagneticHeading3_.SetRecordForDebug(true);
  CRte::GetInstance().oPortSafeMagneticHeading_.SetRecordForDebug(true);
}

static bool IsRecomputedPort(CRte::EPortIds ePortId)
{
  bool bIsRecomputed = false;

  switch (ePortId)
  {
    case CRte::EPortIds::eBahrsFilterOutput1:
      // fallthrough
    case CRte::EPortIds::eBahrsFilterOutput2:
      // fallthrough
    case CRte::EPortIds::eBahrsFilterOutput3:
      // fallthrough
    case CRte::EPortIds::eImuDataAfterMonitor:
      // fallthrough
    case CRte::EPortIds::eImuOutput:
      // fallthrough
    case CRte::EPortIds::eSafePressureData1:
      // fallthrough
    case CRte::EPortIds::eSafePressureData2:
      // fallthrough
    case CRte::EPortIds::eSafePressureData3:
      // fallthrough
    case CRte::EPortIds::eCompensatedPressureInput1:
      // fallthrough
    case CRte::EPortIds::eCompensatedPressureInput2:
      // fallthrough
    case CRte::EPortIds::eCompensatedPressureInput3:
      // fallthrough
    case CRte::EPortIds::eImuInputFiltered1:
      // fallthrough
    case CRte::EPortIds::eImuInputFiltered2:
      // fallthrough
    case CRte::EPortIds::eImuInputFiltered3:
      // fallthrough
    case CRte::EPortIds::eVehicleAttitude1:
      // fallthrough
    case CRte::EPortIds::eVehicleAttitude2:
      // fallthrough
    case CRte::EPortIds::eVehicleAttitude3:
      // fallthrough
    case CRte::EPortIds::eSafeVehicleAttitude:
      // fallthrough
    case CRte::EPortIds::eSafeVerticalChannelData:
      // fallthrough
    case CRte::EPortIds::eCompensatedMagnetometerData1:
      // fallthrough
    case CRte::EPortIds::eCompensatedMagnetometerData2:
      // fallthrough
    case CRte::EPortIds::eCompensatedMagnetometerData3:
      // fallthrough
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame1:
      // fallthrough
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame2:
      // fallthrough
    case CRte::EPortIds::eCompensatedMagnetometerDataInVehicleFrame3:
      // fallthrough
    case CRte::EPortIds::eMagneticHeading1:
      // fallthrough
    case CRte::EPortIds::eMagneticHeading2:
      // fallthrough
    case CRte::EPortIds::eMagneticHeading3:
      // fallthrough
    case CRte::EPortIds::eSafeMagneticHeading:
    {
      bIsRecomputed = true;
      break;
    }
    default:
    {
      bIsRecomputed = false;
      break;
    }
  }

  return bIsRecomputed;
}
