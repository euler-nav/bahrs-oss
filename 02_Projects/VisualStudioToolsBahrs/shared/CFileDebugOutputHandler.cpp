/// @file CFileDebugOutputHandler.cpp
/// @brief Implementation of the file-backed debug output handler for PC replay tools.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CFileDebugOutputHandler.h"
#include "Crc32.h"

#include <cstring>

CFileDebugOutputHandler& CFileDebugOutputHandler::GetInstance()
{
  static CFileDebugOutputHandler soInstance;
  return soInstance;
}

bool CFileDebugOutputHandler::Init(const std::string& korFilePath)
{
  if (bIsInitialized_)
  {
    return true;
  }

  oFile_.open(korFilePath, std::ios::binary | std::ios::out | std::ios::trunc);
  bIsInitialized_ = oFile_.is_open();
  return bIsInitialized_;
}

bool CFileDebugOutputHandler::IsInitialized() const
{
  return bIsInitialized_;
}

void CFileDebugOutputHandler::SendDebugData(const CSerialProtocol::SDebugData& korDebugData)
{
  if (!bIsInitialized_)
  {
    return;
  }

  std::vector<uint8_t> oMessage = buildDebugMessage(korDebugData);

  if (!oMessage.empty())
  {
    oFile_.write(reinterpret_cast<const char*>(oMessage.data()), static_cast<std::streamsize>(oMessage.size()));
    oFile_.flush();
  }
}

std::vector<uint8_t> CFileDebugOutputHandler::buildDebugMessage(const CSerialProtocol::SDebugData& korDebugData) const
{
  unsigned uMessageLen = sizeof(CSerialProtocol::SMessageHeader);

  if (korDebugData.uMsgType_ == CSerialProtocol::EMessageIds::eDebugEventWriteToPort)
  {
    uMessageLen += sizeof(CSerialProtocol::SWriteToPortEventPartialData);
    uMessageLen += static_cast<unsigned>(korDebugData.oByteVector_.size());
    uMessageLen += (PROTOCOL_WORD_LEN - (uMessageLen % PROTOCOL_WORD_LEN));
    uMessageLen += sizeof(CSerialProtocol::CrcType_t);
  }
  else if (korDebugData.uMsgType_ == CSerialProtocol::EMessageIds::eDebugEventRunnableCall)
  {
    uMessageLen += sizeof(CSerialProtocol::SRunnableCallEventData);
    uMessageLen += (PROTOCOL_WORD_LEN - (uMessageLen % PROTOCOL_WORD_LEN));
    uMessageLen += sizeof(CSerialProtocol::CrcType_t);
  }
  else
  {
    return {};
  }

  std::vector<uint8_t> oMessage(uMessageLen, 0U);
  uint8_t* upCopyAddress = oMessage.data();

  CSerialProtocol::SMessageHeader oHeader;
  oHeader.uMsgType_ = static_cast<uint8_t>(korDebugData.uMsgType_);

  putBytes(&upCopyAddress, reinterpret_cast<uint8_t*>(&oHeader), sizeof(oHeader));
  putBytes(&upCopyAddress, &korDebugData.uId_, sizeof(korDebugData.uId_));
  putBytes(&upCopyAddress, reinterpret_cast<const uint8_t*>(&korDebugData.uTimestampUs_), sizeof(korDebugData.uTimestampUs_));

  if (korDebugData.uMsgType_ == CSerialProtocol::EMessageIds::eDebugEventWriteToPort)
  {
    uint16_t uDataLen = static_cast<uint16_t>(korDebugData.oByteVector_.size());
    putBytes(&upCopyAddress, reinterpret_cast<uint8_t*>(&uDataLen), sizeof(uDataLen));

    if (!korDebugData.oByteVector_.empty())
    {
      putBytes(&upCopyAddress, korDebugData.oByteVector_.data(), korDebugData.oByteVector_.size());
    }
  }
  else
  {
    putBytes(&upCopyAddress, &korDebugData.bAllPortsLocked_, sizeof(korDebugData.bAllPortsLocked_));
  }

  const uint32_t uCrc = Crc32(reinterpret_cast<const uint32_t*>(oMessage.data()),
                                     (uMessageLen / PROTOCOL_WORD_LEN) - 1U);
  uint8_t* upCrcAddress = oMessage.data() + (uMessageLen - sizeof(uCrc));
  putBytes(&upCrcAddress, reinterpret_cast<const uint8_t*>(&uCrc), sizeof(uCrc));

  return oMessage;
}


void CFileDebugOutputHandler::putBytes(uint8_t** upAddress, const uint8_t* upData, size_t uLen)
{
  memcpy(*upAddress, upData, uLen);
  *upAddress += uLen;
}
