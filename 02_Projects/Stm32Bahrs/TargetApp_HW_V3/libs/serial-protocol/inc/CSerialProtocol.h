/**
 * @file CSerialProtocol.h
 * @brief Declaration of the BAHRS serial protocol
 * @author Fedor Baklanov
 * @date 07 June 2022
 */

#ifndef C_SERIAL_PROTOCOL_H
#define C_SERIAL_PROTOCOL_H

#include <stdint.h>
#include "CMathConstants.h"
#include "CGeoConstants.h"
#include <limits>
#include <vector>

#define NVM_PAGE_SIZE (32U)

#define PROTOCOL_WORD_LEN (4)
#define PADDING_SIZE(SPayloadType) (PROTOCOL_WORD_LEN - ((sizeof(SMessageHeader) + sizeof(SPayloadType)) % PROTOCOL_WORD_LEN))

#define BIT_POS_HEALTH_HEIGHT           (0U)
#define BIT_POS_HEALTH_VELOCITY_DOWN    (2U)
#define BIT_POS_HEALTH_ROLL             (4U)
#define BIT_POS_HEALTH_PITCH            (6U)
#define BIT_POS_HEALTH_MAGNETIC_HEADING (8U)
#define BIT_POS_HEALTH_SPECIFIC_FORCE_X (0U)
#define BIT_POS_HEALTH_SPECIFIC_FORCE_Y (2U)
#define BIT_POS_HEALTH_SPECIFIC_FORCE_Z (4U)
#define BIT_POS_HEALTH_ANGULAR_RATE_X   (6U)
#define BIT_POS_HEALTH_ANGULAR_RATE_Y   (8U)
#define BIT_POS_HEALTH_ANGULAR_RATE_Z   (10U)

using NNavigationUtilities::CMathConstants;
using NNavigationUtilities::CGeoConstants;

/**
 * @brief The class that describes and implements the BAHRS serial protocol.
*/
class CSerialProtocol
{
public:
  CSerialProtocol () = default;
  ~CSerialProtocol () = default;

  static constexpr uint8_t uMarker1_ { 0x4E }; ///< Sync byte 1, symbol 'N'
  static constexpr uint8_t uMarker2_ { 0x45 }; ///< Sync char 2, symbol 'E'
  static constexpr uint16_t uVersion_ { 3U }; ///< Protocol version

  // Signal ranges
  static constexpr float skfMaxHeight_ { 10000.0F };
  static constexpr float skfMinHeight_ { -1000.0F };
  static constexpr float skfMaxVelocityDown_ { 300.0F };
  static constexpr float skfMinVelocityDown_ { -300.0F };
  static constexpr float skfMaxAngularRate_ { 300.0F * (CMathConstants::skfPi_ / 180.0F) }; ///< +-300 [deg/s] in [rad/s]
  static constexpr float skfMaxSpecificForce_ { CGeoConstants::skfGravity * 5.0F }; ///< +-5 [g] in [m/s^2]

  static constexpr float skfSpecificForceScale_ { skfMaxSpecificForce_ / static_cast<float>(std::numeric_limits<int16_t>::max()) }; ///< Integer to float, +-5g range
  static constexpr float skfAngularRateScale_ { skfMaxAngularRate_ / static_cast<float>(std::numeric_limits<int16_t>::max()) }; ///< Integer to float, +-300 deg/s range
  static constexpr float skfHeightScale_ { 0.16784924F }; ///< Integer to float, -1000 to 10000 m range
  static constexpr float skfHeighOffset_ { 1000.0F }; ///< An offset to convert unsigned integer to float
  static constexpr float skfVelocityDownScale_ { 9.155413e-3F }; ///< Integer to float, -300 to 300 m/s range
  static constexpr float skfAngleScale_ { 9.587526e-5F }; ///< Integer to float, -pi to pi or 0 to 2 pi range

  enum class EMessageIds : uint8_t
  {
    eInvalid = 0x00, ///< Invalid
    eInertialData = 0x01, ///< Inertial data message
    eNavigationData = 0x02, ///< Navigation data message
    eAccuracy = 0x03, ///< Attitude accuracy information
    eTimeOfNavigationData = 0x04, ///< "Time of navigation data" message
    eTimeOfInertialData = 0x05, ///< "Time of inertial data" message
    eTimeOfSyncPulse = 0x06, ///< "Time of the latest sync pulse" message
    eSoftwareVersion = 0x0F, ///< "Software version" message
    eHardwareVersion = 0x1F, ///< "Hardware version" message

    eDebugEventWriteToPort = 0xC0, ///< Debug information: SWC port data
    eDebugEventRunnableCall = 0xC1, ///< Debug information: SWC API call

    eTypeOpenDiagnostic = 0xF0, ///< Request to enter diagnostics mode
    eTypeCloseDiagnostic = 0xF1, ///< Request to exit diagnostics mode
    eTypeReadNVMPage = 0xF2, ///< Request to read NVM page
    eTypeNVMPageData = 0xF3, ///< A message with NVM page data
    eTypeAccept = 0xFF ///< Acknowledgment of diagnostics message reception
  };

  enum class ESignalHealthInfo : uint8_t
  {
    eUnavailable = 0U,
    eSafe,
    eIntegrityRisk
  };

  using CrcType_t = uint32_t;

#pragma pack(push, 1)

  /**
   * @brief Message header struct.
  */
  struct SMessageHeader
  {
    uint8_t uMarker1_ { 0x4E };
    uint8_t uMarker2_ { 0x45 };
    uint16_t uVersion_ { CSerialProtocol::uVersion_ };
    uint8_t uMsgType_ { 0U };
  };

  /**
   * @brief Inertial data message payload.
  */
  struct SInertialData
  {
    uint8_t uSequenceCounter_ { 0U };
    int16_t iSpecificForceX_ { 0 };
    int16_t iSpecificForceY_ { 0 };
    int16_t iSpecificForceZ_ { 0 };
    int16_t iAngularRateX_ { 0 };
    int16_t iAngularRateY_ { 0 };
    int16_t iAngularRateZ_ { 0 };
    uint16_t uHealthInfo_ { 0U };
  };

  /**
   * @brief Payload of the time of inertial data message.
  */
  struct STimeOfInertialData
  {
    uint8_t uSequenceCounter_ { 0U };
    uint8_t uInertialDataSequenceCounter_ { 0U };
    uint64_t uTimestampUs_ { 0U };
  };

  /**
   * @brief Navigation data message payload.
  */
  struct SNavigationData
  {
    uint8_t uSequenceCounter_ { 0U };
    uint16_t uPressureHeight_ { 0 };
    int16_t iVelocityDown_ { 0 };
    int16_t iRoll_ { 0 };
    int16_t iPitch_ { 0 };
    uint16_t uMagneticHeading_ { 0U };
    uint16_t uHealthInfo_ { 0 };
  };

  /**
   * @brief Payload of the "time of navigation data" message. 
  */
  struct STimeOfNavigationData
  {
    uint8_t uSequenceCounter_ { 0U };
    uint8_t uNavigationDataSequenceCounter_ { 0U };
    uint64_t uTimestampUs_ { 0U };
  };

  /**
   * @brief Accuracy message payload.
  */
  struct SAccuracyData
  {
    uint8_t uSequenceCounter_ { 0U };
    uint16_t uAttitudeStdN_ { 0U };
    uint16_t uAttitudeStdE_ { 0U };
    uint16_t uMagneticHeadingStd_ { 0U };
    uint64_t uTimestampUs_ { 0U };
  };

  /**
   * @brief Payload of the "time of the latest pulse" message.
  */
  struct STimeOfLatestSyncPulse
  {
    uint8_t uSequenceCounter_ { 0U };
    uint64_t uTimestampUs_ { 0U };
  };

  /**
   * @brief Payload of the "software version" message.
  */
  struct SSoftwareVersionData
  {
    char acProjectCode_[3] { '\0', '\0', '\0' };
    uint16_t uMajor_ { 0U };
    uint16_t uMinor_ { 0U };
  };

  /**
   * @brief Payload of the "Hardware version" message.
   */
  struct SHardwareVersionData
  {
    uint16_t uMcuId_{0U};
    uint32_t uUniqueId1_{0U};
    uint32_t uUniqueId2_{0U};
    uint32_t uUniqueId3_{0U};
  };

  /**
   * @brief Partial data of the "write to port event".
   * The struct does not include a variable size array of data written to a port.
   */
  struct SWriteToPortEventPartialData
  {
    uint8_t uPortId_ { 0U };
    uint64_t uTimestampUs_ { 0U };
    uint16_t uDataLen_ { 0U };
  };

  /**
   * @brief Debug information on runnable call.
   */
  struct SRunnableCallEventData
  {
    uint8_t uRunnableId_ { 0U };
    uint64_t uTimestampUs_ { 0U };
    uint8_t bAllPortsLocked_ { 0U };
  };

  /**
   * @brief Inertial data message.
  */
  struct SInertialDataMessage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eInertialData) };

    SInertialData oInertialData_;
    uint8_t auPadding_[PADDING_SIZE(SInertialData)];
    CrcType_t uCrc_ { 0U };
  };

  /**
   * @brief Time of inertial data message.
  */
  struct STimeOfInertialDataMessage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eTimeOfInertialData) };

    STimeOfInertialData oTimeOfInertialData_;
    uint8_t auPadding_[PADDING_SIZE(STimeOfInertialData)];
    CrcType_t uCrc_ { 0U };
  };

  /**
   * @brief Navigation data message.
  */
  struct SNavigationDataMessage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eNavigationData) };

    SNavigationData oNavigationData_;
    uint8_t auPadding_[PADDING_SIZE(SNavigationData)];
    CrcType_t uCrc_ { 0U };
  };

  /**
   * @brief Navigation data accuracy message.
  */
  struct SAccuracyDataMessage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eAccuracy) };

    SAccuracyData oAccuracy_;
    uint8_t auPadding_[PADDING_SIZE(SAccuracyData)];
    CrcType_t uCrc_ { 0U };
  };

  /**
   * @brief Time of navigation data message.
  */
  struct STimeOfNavigationDataMessage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eTimeOfNavigationData) };

    STimeOfNavigationData oTimeOfNavigationData_;
    uint8_t auPadding_[PADDING_SIZE(STimeOfNavigationData)];
    CrcType_t uCrc_ { 0U };
  };

  /**
   * @brief Time of the latest sync pulse message.
  */
  struct STimeOfLatestSyncPulseMessage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eTimeOfSyncPulse) };

    STimeOfLatestSyncPulse oTimeOfLatestSyncPulse_;
    uint8_t auPadding_[PADDING_SIZE(STimeOfLatestSyncPulse)];
    CrcType_t uCrc_ { 0U };
  };

  /**
   * @brief Software version message.
  */
  struct SSoftwareVersionMessage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eSoftwareVersion) };

    SSoftwareVersionData oSoftwareVersion_;
    uint8_t auPadding_[PADDING_SIZE(SSoftwareVersionData)];
    CrcType_t uCrc_ { 0U };
  };

  struct SHardwareVersionMessage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eHardwareVersion) };

    SHardwareVersionData oHardwareVersion_;
    uint8_t auPadding_[PADDING_SIZE(SHardwareVersionData)];
    CrcType_t uCrc_ { 0U };
  };

  /**
   * @brief A debug message with runnable call data.
  */
  struct SRunnableCallEventDebugMessage
  {
    SMessageHeader oHeader_{ CSerialProtocol::uMarker1_,
                             CSerialProtocol::uMarker2_,
                             CSerialProtocol::uVersion_,
                             static_cast<uint8_t>(EMessageIds::eDebugEventRunnableCall) };

    SRunnableCallEventData oRunnableCallData_;
    uint8_t auPadding_[PADDING_SIZE(oRunnableCallData_)];
    CrcType_t uCrc_{ 0U };
  };

  /**
   * @brief Diagnostic Mode Messages
   ****************************************************************************/

  /**
   * @brief Diagnostic message: Page request from NVM.
  */
  struct SPacketReadNVMPage
  {
    SMessageHeader oHeader_;
    uint8_t uPageNumber { 0U };
    CrcType_t uCrc32 { 0U };
  };

  /**
   * @brief Diagnostic message: confirmation of request processing by the device.
  */
  struct SPacketReceiveConfirmation
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eTypeAccept) };
    EMessageIds eMessageType;    // message type to confirm.
    uint8_t uStatus { 0U };      // 0 - OK, other values - error codes
    CrcType_t uCrc32 { 0U };
  };

  /**
   * @brief Diagnostic message: One NVM page ( both: RX & TX )
  */
  struct SPacketNVMPage
  {
    SMessageHeader oHeader_ { CSerialProtocol::uMarker1_,
                              CSerialProtocol::uMarker2_,
                              CSerialProtocol::uVersion_,
                              static_cast<uint8_t>(EMessageIds::eTypeNVMPageData) };
    uint8_t uPageNumber { 0U };
    uint8_t auPageData[NVM_PAGE_SIZE];
    CrcType_t uCrc32;
  };

#pragma pack(pop)

};

#endif /* C_SERIAL_PROTOCOL_H */
