/**
 * @file TaskRoutines.cpp
 * @brief Implementation of FreeRTOS task routines that wrap C++ code into C-functions.
 * @author Fedor Baklanov
 * @date 05 February 2022
 */

#include "CRte.h"
#include <assert.h>
#include "GetMicroseconds.h"
#include "cmsis_os.h"

extern "C" void TaskRoutine1ms()
{
  // Nothing to do
}

extern "C" void TaskRoutine5ms()
{
  static uint32_t suCounter = 0U;

  // Send the software version message one time.
  if (0U == suCounter)
  {
    CRs232OutputHandler::GetInstance().QueueTransmissionRequest(CSerialProtocol::EMessageIds::eSoftwareVersion);
  }

  // Poll the BMP384 data at 25 Hz
  if (0U == (suCounter % 8U))
  {
    CBmp384Driver::GetInstance().PollSensor();
    CBahrsFilterSwc::GetInstance().SetPressureInput();
  }

  CScha63TDriver::GetInstance().ConvertRawDataset();
  CImuMonitorSwc::GetInstance().Run();
  CBahrsFilterSwc::GetInstance().SetImuInput();

  // Do the following task at 10Hz
  if (0U == (suCounter % 20U))
  {
    CMmc5983Driver::GetInstance().PollSensor();
    CBmm150Driver::GetInstance(0).Bmm150ReadMagData();
    CBmm150Driver::GetInstance(1).Bmm150ReadMagData();
  }

  COutputTransformer::GetInstance().TransformImuSignals();

  // Request to send inertial data message
  CRs232OutputHandler::GetInstance().QueueTransmissionRequest(CSerialProtocol::EMessageIds::eInertialData);

  // Request to send time of inertial data at 10Hz
  if (0U == (suCounter % 20U))
  {
    CRs232OutputHandler::GetInstance().QueueTransmissionRequest(CSerialProtocol::EMessageIds::eTimeOfInertialData);
  }

  ++suCounter;
}

extern "C" void TaskRoutine10ms()
{
  static uint32_t suCounter = 0U;

  CBahrsFilterSwc::GetInstance().Step(GetMicroseconds());

  // Request to send navigation data message
  CRs232OutputHandler::GetInstance().QueueTransmissionRequest(CSerialProtocol::EMessageIds::eNavigationData);

  // Request to send some information at 10Hz
  if (0U == (suCounter % 10U))
  {
    CRs232OutputHandler::GetInstance().QueueTransmissionRequest(CSerialProtocol::EMessageIds::eAccuracy);
    CRs232OutputHandler::GetInstance().QueueTransmissionRequest(CSerialProtocol::EMessageIds::eTimeOfNavigationData);
  }

  ++suCounter;
}

extern "C" void TaskRoutineRs232Sender(uint8_t uMessageId)
{
  CSerialProtocol::EMessageIds eMessageId = static_cast<CSerialProtocol::EMessageIds>(uMessageId);

  switch (eMessageId)
  {
    case CSerialProtocol::EMessageIds::eInertialData:
      CRs232OutputHandler::GetInstance().SendInertialDataMessage();
      break;
    case CSerialProtocol::EMessageIds::eTimeOfInertialData:
      CRs232OutputHandler::GetInstance().SendTimeOfInertialDataMessage();
      break;
    case CSerialProtocol::EMessageIds::eNavigationData:
      CRs232OutputHandler::GetInstance().SendNavigationDataMessage();
      break;
    case CSerialProtocol::EMessageIds::eTimeOfNavigationData:
      CRs232OutputHandler::GetInstance().SendTimeOfNavigationDataMessage();
      break;
    case CSerialProtocol::EMessageIds::eAccuracy:
      CRs232OutputHandler::GetInstance().SendAccuracyDataMessage();
      break;
    case CSerialProtocol::EMessageIds::eTimeOfSyncPulse:
      CRs232OutputHandler::GetInstance().SendTimeOfLatestSyncPulseMessage();
      break;
    case CSerialProtocol::EMessageIds::eSoftwareVersion:
      CRs232OutputHandler::GetInstance().SendSoftwareVersionMessage();
      break;
    default:
      break;
  }
}

extern "C" void InitializeSensors()
{
  bool bStatus;

  CBmp384Driver::GetInstance().Init();
  bStatus = CBmp384Driver::GetInstance().IsInitialized();

  if (true == bStatus)
  {
    CMmc5983Driver::GetInstance().Init();
    bStatus = CMmc5983Driver::GetInstance().IsInitialized();
  }

  if (true == bStatus)
  {
    CScha63TDriver::GetInstance().Init();
    bStatus = CScha63TDriver::GetInstance().IsInitialized();
  }

  if (true == bStatus)
  {
    CBmm150Driver::GetInstance(0).Init();
    bStatus = CBmm150Driver::GetInstance(0).IsInitialized();
  }

  if (true == bStatus)
  {
    CBmm150Driver::GetInstance(1).Init();
    bStatus = CBmm150Driver::GetInstance(1).IsInitialized();
  }

  if (true == bStatus)
  {
    CIcm20789Driver::GetInstance(0).Init();
    bStatus = CIcm20789Driver::GetInstance(0).IsInitialized();
  }

  if (true == bStatus)
  {
    CIcm20789Driver::GetInstance(1).Init();
    bStatus = CIcm20789Driver::GetInstance(1).IsInitialized();
  }

  if (true == bStatus)
  {
    COutputTransformer::GetInstance().Init();
    bStatus = COutputTransformer::GetInstance().IsInitialized();
  }

  if (false == bStatus)
  {
    // If some driver failed to initialize, the SW shall not continue.
    // TODO: implement failure handling
    assert(false);
  }
}

extern "C" void TaskRoutineProcessSyncPulse(uint64_t uTimestampUs)
{
  CSyncPulseHandler::ProcessSyncPulse(uTimestampUs);
}

