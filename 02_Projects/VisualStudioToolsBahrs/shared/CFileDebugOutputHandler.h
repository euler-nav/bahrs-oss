/// @file CFileDebugOutputHandler.h
/// @brief Declaration of the file-backed debug output handler for PC replay tools.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_FILE_DEBUG_OUTPUT_HANDLER_H
#define C_FILE_DEBUG_OUTPUT_HANDLER_H

#include "CSerialProtocol.h"
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

class CFileDebugOutputHandler
{
public:
  /// @brief Get the singleton instance.
  /// @return Reference to the singleton instance.
  static CFileDebugOutputHandler& GetInstance();

  /// @brief Initialize the debug output file.
  /// @param korFilePath Output file path.
  /// @return True if the file is open or already initialized.
  bool Init(const std::string& korFilePath);

  /// @brief Check whether the handler is initialized.
  /// @return True if initialized.
  bool IsInitialized() const;

  /// @brief Serialize and write one debug message.
  /// @param korDebugData Debug data to serialize.
  void SendDebugData(const CSerialProtocol::SDebugData& korDebugData);

private:
  CFileDebugOutputHandler() = default;
  ~CFileDebugOutputHandler() = default;

  CFileDebugOutputHandler(const CFileDebugOutputHandler&) = delete;
  CFileDebugOutputHandler& operator=(const CFileDebugOutputHandler&) = delete;

  std::vector<uint8_t> buildDebugMessage(const CSerialProtocol::SDebugData& korDebugData) const;
  static void putBytes(uint8_t** upAddress, const uint8_t* upData, size_t uLen);

  std::ofstream oFile_{};
  bool bIsInitialized_{ false };
};

#endif // C_FILE_DEBUG_OUTPUT_HANDLER_H
