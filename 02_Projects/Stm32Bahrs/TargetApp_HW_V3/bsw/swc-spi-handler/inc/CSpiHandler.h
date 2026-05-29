/// @file CSpiHandler.h
/// @brief Declaration of the SPI handler software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_SPI_HANDLER_H
#define C_SPI_HANDLER_H

#include "General/CSoftwareComponentBase.h"
#include "spi.h"
#include "CMutex.h"

/// @brief Software component class for handling SPI communication with peripherals.
class CSpiHandler : public CSoftwareComponent<CSpiHandler, 1U>
{
  friend class CSoftwareComponent<CSpiHandler, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CSpiHandler)
  DECLARE_MANDATORY_APIS(CSpiHandler)

public:
  /// @brief Enumeration of supported peripherals connected via SPI.
  enum class EPeripheral : uint8_t
  {
    eBmi270,
    eBmp384,
    eAsm330,
    eLps22
  };

  /// @brief Transmit data to a specified peripheral over SPI.
  /// @param ePeripheral The peripheral to transmit data to.
  /// @param uRegAddr The address of the register to write to.
  /// @param uRegAddrLen The length of the register address in bytes.
  /// @param upData The data to transmit.
  /// @param uSize The size of the data to transmit.
  /// @param uTimeout The timeout for the transmission in OS ticks.
  /// @return True if the transmission was successful, false otherwise.
  bool Transmit(EPeripheral ePeripheral, const uint8_t* uRegAddr, uint8_t uRegAddrLen, const uint8_t* upData, uint16_t uSize, uint32_t uTimeout);

  /// @brief Receive data from a specified peripheral over SPI.
  /// @param ePeripheral The peripheral to receive data from.
  /// @param uRegAddr The address of the register to read from.
  /// @param uRegAddrLen The length of the register address in bytes.
  /// @param upData The buffer to store the received data.
  /// @param uSize The size of the data to receive.
  /// @param uTimeout The timeout for the reception in OS ticks.
  /// @return True if the reception was successful, false otherwise.
  bool Receive(EPeripheral ePeripheral, const uint8_t* uRegAddr, uint8_t uRegAddrLen, uint8_t* upData, uint16_t uSize, uint32_t uTimeout);


private:
  CSpiHandler() = default;
  ~CSpiHandler() = default;

  static constexpr uint32_t skuMutexAcquisitionTimeout_{ 2U }; ///< Timeout for acquiring the SPI mutex in OS ticks.

  /// @brief Internal method to handle SPI transmission.
  /// @param opSpiHandle The SPI handle to use for transmission.
  /// @param opGpioPort The GPIO port for the chip select pin.
  /// @param uChipSelectPin The chip select pin number.
  /// @param uRegAddr The address of the register to write to.
  /// @param uRegAddrLen The length of the register address in bytes.
  /// @param upData The data to transmit.
  /// @param uSize The size of the data to transmit.
  /// @param uTimeout The timeout for the transmission in OS ticks.
  /// @return True if the transmission was successful, false otherwise.
  bool transmitInternal(SPI_HandleTypeDef* opSpiHandle, GPIO_TypeDef* opGpioPort, uint16_t uChipSelectPin, const uint8_t* uRegAddr, uint8_t uRegAddrLen, const uint8_t* upData, uint16_t uSize, uint32_t uTimeout);
  
  /// @brief Internal method to handle SPI reception.
  /// @param opSpiHandle The SPI handle to use for reception.
  /// @param opGpioPort The GPIO port for the chip select pin.
  /// @param uChipSelectPin The chip select pin number.
  /// @param uRegAddr The address of the register to read from.
  /// @param uRegAddrLen The length of the register address in bytes.
  /// @param upData The buffer to store the received data.
  /// @param uSize The size of the data to receive.
  /// @param uTimeout The timeout for the reception in OS ticks.
  /// @return True if the reception was successful, false otherwise.
  bool receiveInternal(SPI_HandleTypeDef* opSpiHandle, GPIO_TypeDef* opGpioPort, uint16_t uChipSelectPin, const uint8_t* uRegAddr, uint8_t uRegAddrLen, uint8_t* upData, uint16_t uSize, uint32_t uTimeout);
  
  /// @brief Acquire the mutex for the specified SPI handle.
  /// @param opSpiHandle The SPI handle for which to acquire the mutex.
  /// @return True if the mutex was successfully acquired, false otherwise.
  bool mutexAcquire(const SPI_HandleTypeDef* opSpiHandle);

  /// @brief Release the mutex for the specified SPI handle.
  /// @param opSpiHandle The SPI handle for which to release the mutex.
  void mutexRelease(const SPI_HandleTypeDef* opSpiHandle);

  CMutex oMutexSpi4_;
  CMutex oMutexSpi1_;
  bool bIsInitialized_{false};

};

#endif // C_SPI_HANDLER_H

