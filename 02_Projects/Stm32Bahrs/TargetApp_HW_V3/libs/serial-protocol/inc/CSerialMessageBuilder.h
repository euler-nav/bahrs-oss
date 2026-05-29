/**
 * @file CSerialMessageBuilder.h
 * @brief Declaration of the BAHRS serial protocol message builder class.
 * @author Fedor Baklanov
 * @date 27 November 2024
 */

#ifndef C_SERIAL_MESSAGE_BUILDER_H
#define C_SERIAL_MESSAGE_BUILDER_H

#include "CSerialProtocol.h"
#include "RteTypesGenerated.h"
#include "RteTypesUserDefined.h"
#include "CMutex.h"

/**
 * @brief A class for building messages according to BAHRS serial protocol.
*/
class CSerialMessageBuilder
{
public:
  CSerialMessageBuilder();

  /**
   * @brief Build inertial data message from IMU measurements.
   * @param korImuData Reference to IMU data.
   * @return Inertial data message struct.
  */
  CSerialProtocol::SInertialDataMessage BuildInertialDataMessage(const SOutputImuData& korImuData);

  /**
   * @brief Build the "Time of inertial data message" from IMU measurements.
   * @param korImuData Reference to IMU measurement.
   * @return "Time of inertial data message" struct.
  */
  CSerialProtocol::STimeOfInertialDataMessage BuildTimeOfInertialDataMessage(const SOutputImuData& korImuData);

  /**
   * @brief Build navigation data message.
   * @param korVerticalChannelData Reference to vertical channel data.
   * @param korVehicleAttitude Reference to vehicle attitude data.
   * @param korMagneticHeading Reference to the estimated magnetic heading data compensated for installation alignment
   * @return Navigation data message struct.
  */
  CSerialProtocol::SNavigationDataMessage BuildNavigationDataMessage(const SSafeVerticalChannelData& korVerticalChannelData,
                                                                     const SSafeAttitudeData& korVehicleAttitude,
                                                                     const SMagneticHeading& korMagneticHeading);

  /**
   * @brief Build time of navigation data message.
   * @param korVehicleAttitude Reference to vehicle attitude data.
   * @return Time of navigation data message.
  */
  CSerialProtocol::STimeOfNavigationDataMessage BuildTimeOfNavigationDataMessage(const SSafeAttitudeData& korVehicleAttitude);

  /**
   * @brief Build navigation data accuracy message.
   * @param korVehicleAttitude Reference to vehicle attitude data.
   * @return Accuracy data message.
  */
  CSerialProtocol::SAccuracyDataMessage BuildAccuracyDataMessage(const SSafeAttitudeData& korVehicleAttitude);

  /**
   * @brief Build time of the latest sync pulse message.
   * @param uTimestamp Measured time of the latest sync pulse.
   * @return The time of the latest sync pulse message.
  */
  CSerialProtocol::STimeOfLatestSyncPulseMessage BuildTimeOfLatestSyncPulseMessage(uint64_t uTimestamp);

  /**
   * @brief Build the software version message.
   * @return The software version message struct.
   */
  CSerialProtocol::SSoftwareVersionMessage BuildSoftwareVersionMessage();

  /**
   * @brief Build the hardware version message.
   * @return The hardware version message struct.
   */
  CSerialProtocol::SHardwareVersionMessage BuildHardwareVersionMessage();

  /**
   * @brief Calculate CRC.
   * @param pData Pointer to 32-bit data words aarray.
   * @param uNumberOfWords_ Number of words in the array.
   * @return 32-bit CRC (CRC-32/MPEG2)
  */
  static uint32_t CalculateCrc(uint32_t* pData, uint32_t uNumberOfWords_);

protected:

private:
  static constexpr uint32_t skuCrcMutexTimeoutInTicks_{20U};
  static CMutex oCrcMutex_;

  uint8_t uInertialDataSequenceCounter_ { 0U }; ///< Sent inertial data message count.
  uint8_t uNavigationDataSequenceCounter_ { 0U }; ///< Navigation inertial data message count.
  uint8_t uAccuracyDataSequenceCounter_ { 0U }; ///< Sent accuracy message count.
  uint8_t uTimeOfInertialDataSequenceCounter_ { 0U }; ///< Sent time of inertial data message count.
  uint8_t uTimeOfNavigationDataSequenceCounter_ { 0U }; ///< Sent time of navigaiton data message count.
  uint8_t uTimeOfLatestPulseSequenceCounter_ { 0U }; ///< Sent time of the latest pulse message count.

};

#endif /* C_SERIAL_MESSAGE_BUILDER_H */
