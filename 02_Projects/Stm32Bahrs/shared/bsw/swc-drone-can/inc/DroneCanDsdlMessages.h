/// @file DroneCanDsdlMessages.h
/// @brief Declaration of DroneCAN DSDL message type IDs, signatures, and payload serialization helpers.
///
/// Every message used here is a standard upstream type from https://github.com/dronecan/DSDL.
/// This component defines no custom data types, so no DSDL files are kept in this
/// repository and no code generation runs as part of the build - the serializers
/// below are written by hand.
///
/// Data type signatures must nevertheless match the values that dronecan_dsdlc
/// produces for those upstream definitions.  dronecan_dsdlc is not published on
/// PyPI, so it has to be cloned and run directly:
///   git clone https://github.com/dronecan/dronecan_dsdlc
///   git clone https://github.com/dronecan/DSDL
///   pip install -r dronecan_dsdlc/requirements.txt
///   python dronecan_dsdlc/dronecan_dsdlc.py -O <output-dir> DSDL/uavcan
/// Then compare the *_SIGNATURE macros in the generated headers with the constants
/// below.  Note that "pip install dronecan" installs the Python library behind
/// dronecan_gui_tool, which is a different tool and does not emit C headers.
///
/// Serialization is implemented manually using canardEncodeScalar() and
/// canardConvertNativeFloatToFloat16() from libcanard.  The bit layouts follow the
/// UAVCAN v0 specification (DroneCAN): fields are packed LSB-first with no padding
/// between adjacent fields.  Variable-length (non-tail) arrays are preceded by a
/// ceil(log2(max_len+1))-bit unsigned length field.  The last variable-length array
/// (tail array) carries no length prefix.
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef DRONE_CAN_DSDL_MESSAGES_H
#define DRONE_CAN_DSDL_MESSAGES_H

#include "canardWrapper.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// -------------------------------------------------------------------------
// Data type IDs (from dronecan/DSDL)
// ----------------------------------------------------------------------

/// @brief uavcan.protocol.GetNodeInfo (service)
#define DRONE_CAN_DSDL_PROTOCOL_GET_NODE_INFO_ID     1U

/// @brief uavcan.protocol.NodeStatus
#define DRONE_CAN_DSDL_PROTOCOL_NODE_STATUS_ID       341U

/// @brief uavcan.equipment.ahrs.Solution
#define DRONE_CAN_DSDL_AHRS_SOLUTION_ID              1000U

/// @brief uavcan.equipment.ahrs.RawIMU
#define DRONE_CAN_DSDL_AHRS_RAWIMU_ID               1003U

/// @brief uavcan.equipment.air_data.StaticPressure
#define DRONE_CAN_DSDL_AIR_DATA_STATIC_PRESSURE_ID  1028U

/// @brief uavcan.equipment.air_data.StaticTemperature
#define DRONE_CAN_DSDL_AIR_DATA_STATIC_TEMPERATURE_ID 1029U

// -------------------------------------------------------------------------
// Data type signatures (CRC-64/Jones of canonical DSDL definition)
// Verify against values emitted by dronecan_dsdlc before production use.
// ----------------------------------------------------------------------

/// @brief uavcan.protocol.GetNodeInfo signature
#define DRONE_CAN_DSDL_PROTOCOL_GET_NODE_INFO_SIGNATURE 0xEE468A8121C46A9EULL

/// @brief uavcan.protocol.NodeStatus signature
#define DRONE_CAN_DSDL_PROTOCOL_NODE_STATUS_SIGNATURE 0x0F0868D0C1A7C6F1ULL

/// @brief uavcan.equipment.ahrs.Solution signature (verified against dronecan package)
#define DRONE_CAN_DSDL_AHRS_SOLUTION_SIGNATURE       0x72A63A3C6F41FA9BULL

/// @brief uavcan.equipment.ahrs.RawIMU signature (verified against dronecan package)
#define DRONE_CAN_DSDL_AHRS_RAWIMU_SIGNATURE         0x8280632C40E574B5ULL

/// @brief uavcan.equipment.air_data.StaticPressure signature (verified against dronecan package)
#define DRONE_CAN_DSDL_AIR_DATA_STATIC_PRESSURE_SIGNATURE  0xCDC7C43412BDC89AULL

/// @brief uavcan.equipment.air_data.StaticTemperature signature (verified against dronecan package)
#define DRONE_CAN_DSDL_AIR_DATA_STATIC_TEMPERATURE_SIGNATURE 0x49272A6477D96271ULL

// -------------------------------------------------------------------------
// NodeStatus health / mode enumerations (uavcan.protocol.NodeStatus)
// ----------------------------------------------------------------------

/// @brief NodeStatus.health values (2-bit field).
#define DRONE_CAN_DSDL_NODE_HEALTH_OK        0U
#define DRONE_CAN_DSDL_NODE_HEALTH_WARNING   1U
#define DRONE_CAN_DSDL_NODE_HEALTH_ERROR     2U
#define DRONE_CAN_DSDL_NODE_HEALTH_CRITICAL  3U

/// @brief NodeStatus.mode values (3-bit field).
#define DRONE_CAN_DSDL_NODE_MODE_OPERATIONAL     0U
#define DRONE_CAN_DSDL_NODE_MODE_INITIALIZATION  1U
#define DRONE_CAN_DSDL_NODE_MODE_MAINTENANCE     2U
#define DRONE_CAN_DSDL_NODE_MODE_SOFTWARE_UPDATE 3U
#define DRONE_CAN_DSDL_NODE_MODE_OFFLINE         7U

// -------------------------------------------------------------------------
// Payload sizes (bytes) -- with zero-item variable-length arrays
// ----------------------------------------------------------------------

/// NodeStatus payload: uint32 uptime_sec + uint2 health + uint3 mode +
/// uint3 sub_mode + uint16 vendor_specific_status_code = 56 bits = 7 bytes.
#define DRONE_CAN_DSDL_PROTOCOL_NODE_STATUS_PAYLOAD_SIZE  7U

/// Maximum length of the GetNodeInfo response `name` field (uint8[<=80]).
#define DRONE_CAN_DSDL_GET_NODE_INFO_NAME_MAX             80U

/// GetNodeInfo response maximum payload size:
///   NodeStatus(7) + SoftwareVersion(15) + HardwareVersion(2 + 16 unique_id +
///   1 certificate length prefix) + name(<=80) = 121 bytes.
#define DRONE_CAN_DSDL_GET_NODE_INFO_MAX_PAYLOAD_SIZE    121U

/// Solution payload with no covariance (0-item tail array for linear_acceleration_covariance).
/// Fields: uint56 timestamp + float16[4] orientation_xyzw + void4 + (4-bit len=0)
///         orientation_covariance + float16[3] angular_velocity + void4 + (4-bit len=0)
///         angular_velocity_covariance + float16[3] linear_acceleration + (0-item TAO tail)
///         = 232 bits = 29 bytes.
#define DRONE_CAN_DSDL_AHRS_SOLUTION_PAYLOAD_SIZE    29U

/// RawIMU payload with no covariance (0-item TAO tail array for covariance).
/// Fields: uint56 timestamp + float32 integration_interval + float16[3] rate_gyro_latest
///         + float32[3] rate_gyro_integral + float16[3] accelerometer_latest
///         + float32[3] accelerometer_integral = 47 bytes.
/// NOTE: the integral fields are float32[3] (not float16), and there is no sensor_id field.
#define DRONE_CAN_DSDL_AHRS_RAWIMU_PAYLOAD_SIZE      47U

/// StaticPressure payload: float32 + float16 = 6 bytes.
#define DRONE_CAN_DSDL_AIR_DATA_STATIC_PRESSURE_PAYLOAD_SIZE   6U

/// StaticTemperature payload: float16 + float16 = 4 bytes.
#define DRONE_CAN_DSDL_AIR_DATA_STATIC_TEMPERATURE_PAYLOAD_SIZE 4U

// -------------------------------------------------------------------------
// Serialization helpers
// ----------------------------------------------------------------------

/// @brief Serialize a uavcan.protocol.NodeStatus payload.
///
/// Field layout (LSB-first, no padding), total 56 bits = 7 bytes:
///   uint32 uptime_sec | uint2 health | uint3 mode | uint3 sub_mode |
///   uint16 vendor_specific_status_code
///
/// @param [out] apBuf                        Output buffer, at least
///                                           DRONE_CAN_DSDL_PROTOCOL_NODE_STATUS_PAYLOAD_SIZE bytes.
/// @param [in]  uUptimeSec                   Seconds since boot.
/// @param [in]  uHealth                      DRONE_CAN_DSDL_NODE_HEALTH_* (2-bit).
/// @param [in]  uMode                        DRONE_CAN_DSDL_NODE_MODE_* (3-bit).
/// @param [in]  uSubMode                     Vendor-specific sub-mode (3-bit).
/// @param [in]  uVendorSpecificStatusCode    Vendor-specific status code.
/// @return                                   Payload length in bytes.
static inline uint16_t DroneCanDsdl_SerializeNodeStatus(
    uint8_t* apBuf,
    uint32_t uUptimeSec,
    uint8_t  uHealth,
    uint8_t  uMode,
    uint8_t  uSubMode,
    uint16_t uVendorSpecificStatusCode)
{
    uint32_t uBitOffset = 0U;

    memset(apBuf, 0, DRONE_CAN_DSDL_PROTOCOL_NODE_STATUS_PAYLOAD_SIZE);

    // uint32 uptime_sec
    canardEncodeScalar(apBuf, uBitOffset, 32U, &uUptimeSec); uBitOffset += 32U;

    // uint2 health
    canardEncodeScalar(apBuf, uBitOffset, 2U, &uHealth); uBitOffset += 2U;

    // uint3 mode
    canardEncodeScalar(apBuf, uBitOffset, 3U, &uMode); uBitOffset += 3U;

    // uint3 sub_mode
    canardEncodeScalar(apBuf, uBitOffset, 3U, &uSubMode); uBitOffset += 3U;

    // uint16 vendor_specific_status_code
    canardEncodeScalar(apBuf, uBitOffset, 16U, &uVendorSpecificStatusCode); uBitOffset += 16U;

    return (uint16_t)((uBitOffset + 7U) / 8U);
}

/// @brief Serialize a uavcan.protocol.GetNodeInfo response payload.
///
/// Structure (uavcan.protocol.GetNodeInfo response):
///   uavcan.protocol.NodeStatus     status              (7 bytes)
///   uavcan.protocol.SoftwareVersion software_version   (15 bytes)
///   uavcan.protocol.HardwareVersion hardware_version   (2 + 16 + cert array)
///   uint8[<=80]                    name                (tail array)
///
/// The `certificate_of_authenticity` array inside HardwareVersion is NOT the last
/// field of the message (name follows it), so it carries an 8-bit length prefix
/// (0 here). The `name` array IS the last field, so under DroneCAN tail-array
/// optimization it is written with no length prefix.
///
/// @param [out] apBuf         Output buffer, at least
///                            DRONE_CAN_DSDL_GET_NODE_INFO_MAX_PAYLOAD_SIZE bytes.
/// @param [in]  uUptimeSec    Seconds since boot (NodeStatus.uptime_sec).
/// @param [in]  uHealth       NodeStatus.health (DRONE_CAN_DSDL_NODE_HEALTH_*).
/// @param [in]  uMode         NodeStatus.mode (DRONE_CAN_DSDL_NODE_MODE_*).
/// @param [in]  uSwMajor      SoftwareVersion.major.
/// @param [in]  uSwMinor      SoftwareVersion.minor.
/// @param [in]  uVcsCommit    SoftwareVersion.vcs_commit (0 => flag cleared).
/// @param [in]  uHwMajor      HardwareVersion.major.
/// @param [in]  uHwMinor      HardwareVersion.minor.
/// @param [in]  kpUniqueId    16-byte hardware unique ID.
/// @param [in]  kpName        Node name bytes (e.g. "ams.bahrs").
/// @param [in]  uNameLen      Length of kpName (clamped to 80).
/// @return                    Payload length in bytes.
static inline uint16_t DroneCanDsdl_SerializeGetNodeInfoResponse(
    uint8_t* apBuf,
    uint32_t uUptimeSec,
    uint8_t  uHealth,
    uint8_t  uMode,
    uint8_t  uSwMajor,
    uint8_t  uSwMinor,
    uint32_t uVcsCommit,
    uint8_t  uHwMajor,
    uint8_t  uHwMinor,
    const uint8_t* kpUniqueId,
    const char* kpName,
    uint16_t uNameLen)
{
    uint8_t* p = apBuf;
    uint8_t uOptionalFieldFlags = (uint8_t)((uVcsCommit != 0U) ? 0x01U : 0x00U);

    // uavcan.protocol.NodeStatus status (7 bytes, byte-aligned)
    p += DroneCanDsdl_SerializeNodeStatus(p, uUptimeSec, uHealth, uMode, 0U, 0U);

    // uavcan.protocol.SoftwareVersion (15 bytes)
    *p++ = uSwMajor;                                // uint8 major
    *p++ = uSwMinor;                                // uint8 minor
    *p++ = uOptionalFieldFlags;                     // uint8 optional_field_flags (bit0 = VCS_COMMIT)
    memcpy(p, &uVcsCommit, 4U); p += 4U;            // uint32 vcs_commit
    memset(p, 0, 8U); p += 8U;                      // uint64 image_crc = 0

    // uavcan.protocol.HardwareVersion
    *p++ = uHwMajor;                                // uint8 major
    *p++ = uHwMinor;                                // uint8 minor
    memcpy(p, kpUniqueId, 16U); p += 16U;           // uint8[16] unique_id
    *p++ = 0U;                                      // uint8[<=255] certificate_of_authenticity length prefix (0 items)

    // uint8[<=80] name -- tail array (TAO): raw bytes, no length prefix
    uint16_t uName = uNameLen;
    if (uName > DRONE_CAN_DSDL_GET_NODE_INFO_NAME_MAX)
    {
        uName = DRONE_CAN_DSDL_GET_NODE_INFO_NAME_MAX;
    }
    memcpy(p, kpName, uName); p += uName;

    return (uint16_t)(p - apBuf);
}

/// @brief Serialize a uavcan.equipment.ahrs.Solution payload (no covariance).
///
/// The DroneCAN Solution quaternion field is ordered [X, Y, Z, W].
/// Our internal quaternion representation is [W, X, Y, Z] so the caller
/// must pass components in the correct order.
///
/// @param [out] apBuf            Output buffer, must be at least
///                               DRONE_CAN_DSDL_AHRS_SOLUTION_PAYLOAD_SIZE bytes.
/// @param [in]  uTimestampUs     Timestamp in microseconds (uint56).
/// @param [in]  fQuatX           Quaternion X component.
/// @param [in]  fQuatY           Quaternion Y component.
/// @param [in]  fQuatZ           Quaternion Z component.
/// @param [in]  fQuatW           Quaternion W component.
/// @param [in]  fGyroX           Angular velocity X [rad/s].
/// @param [in]  fGyroY           Angular velocity Y [rad/s].
/// @param [in]  fGyroZ           Angular velocity Z [rad/s].
/// @param [in]  fAccelX          Linear acceleration X [m/s^2].
/// @param [in]  fAccelY          Linear acceleration Y [m/s^2].
/// @param [in]  fAccelZ          Linear acceleration Z [m/s^2].
/// @return                       Payload length in bytes.
static inline uint16_t DroneCanDsdl_SerializeSolution(
    uint8_t* apBuf,
    uint64_t uTimestampUs,
    float fQuatX, float fQuatY, float fQuatZ, float fQuatW,
    float fGyroX, float fGyroY, float fGyroZ,
    float fAccelX, float fAccelY, float fAccelZ)
{
    uint32_t uBitOffset = 0U;
    uint16_t f16;
    uint8_t uZeroLen = 0U;

    memset(apBuf, 0, DRONE_CAN_DSDL_AHRS_SOLUTION_PAYLOAD_SIZE);

    // uint56 timestamp
    canardEncodeScalar(apBuf, uBitOffset, 56U, &uTimestampUs);
    uBitOffset += 56U;

    // float16[4] orientation_xyzw -- DroneCAN field order is X, Y, Z, W
    f16 = canardConvertNativeFloatToFloat16(fQuatX);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;
    f16 = canardConvertNativeFloatToFloat16(fQuatY);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;
    f16 = canardConvertNativeFloatToFloat16(fQuatZ);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;
    f16 = canardConvertNativeFloatToFloat16(fQuatW);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;

    // void4 padding (part of the DSDL, zero)
    uBitOffset += 4U;

    // float16[<=9] orientation_covariance -- 4-bit length prefix, value 0
    canardEncodeScalar(apBuf, uBitOffset, 4U, &uZeroLen); uBitOffset += 4U;

    // float16[3] angular_velocity -- byte-aligned at bit 128 after void4 + length
    f16 = canardConvertNativeFloatToFloat16(fGyroX);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;
    f16 = canardConvertNativeFloatToFloat16(fGyroY);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;
    f16 = canardConvertNativeFloatToFloat16(fGyroZ);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;

    // void4 padding (part of the DSDL, zero)
    uBitOffset += 4U;

    // float16[<=9] angular_velocity_covariance -- 4-bit length prefix, value 0
    canardEncodeScalar(apBuf, uBitOffset, 4U, &uZeroLen); uBitOffset += 4U;

    // float16[3] linear_acceleration -- byte-aligned
    f16 = canardConvertNativeFloatToFloat16(fAccelX);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;
    f16 = canardConvertNativeFloatToFloat16(fAccelY);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;
    f16 = canardConvertNativeFloatToFloat16(fAccelZ);
    canardEncodeScalar(apBuf, uBitOffset, 16U, &f16); uBitOffset += 16U;

    // float16[<=9] linear_acceleration_covariance -- tail array, 0 items, no prefix

    return (uint16_t)((uBitOffset + 7U) / 8U);
}

/// @brief Serialize a uavcan.equipment.ahrs.RawIMU payload (no covariance).
///
/// All fields are byte-aligned when covariance is empty (0-item TAO tail array),
/// so plain byte writes are used.
///
/// Field order (per uavcan.equipment.ahrs.RawIMU):
///   uint56 timestamp | float32 integration_interval |
///   float16[3] rate_gyro_latest | float32[3] rate_gyro_integral |
///   float16[3] accelerometer_latest | float32[3] accelerometer_integral |
///   float16[<=36] covariance (TAO tail, 0 items)
/// NOTE: the *_integral fields are float32[3], and there is no sensor_id field.
///
/// @param [out] apBuf                   Output buffer, at least
///                                      DRONE_CAN_DSDL_AHRS_RAWIMU_PAYLOAD_SIZE bytes.
/// @param [in]  uTimestampUs            Timestamp in microseconds.
/// @param [in]  fIntegrationInterval    Integration interval in seconds.
/// @param [in]  fGyroX                  Rate gyro X latest [rad/s].
/// @param [in]  fGyroY                  Rate gyro Y latest [rad/s].
/// @param [in]  fGyroZ                  Rate gyro Z latest [rad/s].
/// @param [in]  fAccelX                 Accelerometer X latest [m/s^2].
/// @param [in]  fAccelY                 Accelerometer Y latest [m/s^2].
/// @param [in]  fAccelZ                 Accelerometer Z latest [m/s^2].
/// @return                              Payload length in bytes.
static inline uint16_t DroneCanDsdl_SerializeRawImu(
    uint8_t* apBuf,
    uint64_t uTimestampUs,
    float fIntegrationInterval,
    float fGyroX, float fGyroY, float fGyroZ,
    float fAccelX, float fAccelY, float fAccelZ)
{
    uint16_t f16;
    uint8_t* p = apBuf;

    memset(apBuf, 0, DRONE_CAN_DSDL_AHRS_RAWIMU_PAYLOAD_SIZE);

    // uint56 timestamp -- write lowest 7 bytes little-endian
    memcpy(p, &uTimestampUs, 7U); p += 7U;

    // float32 integration_interval
    memcpy(p, &fIntegrationInterval, 4U); p += 4U;

    // float16[3] rate_gyro_latest
    f16 = canardConvertNativeFloatToFloat16(fGyroX); memcpy(p, &f16, 2U); p += 2U;
    f16 = canardConvertNativeFloatToFloat16(fGyroY); memcpy(p, &f16, 2U); p += 2U;
    f16 = canardConvertNativeFloatToFloat16(fGyroZ); memcpy(p, &f16, 2U); p += 2U;

    // float32[3] rate_gyro_integral -- zeroed (no integration implemented)
    p += 12U;

    // float16[3] accelerometer_latest
    f16 = canardConvertNativeFloatToFloat16(fAccelX); memcpy(p, &f16, 2U); p += 2U;
    f16 = canardConvertNativeFloatToFloat16(fAccelY); memcpy(p, &f16, 2U); p += 2U;
    f16 = canardConvertNativeFloatToFloat16(fAccelZ); memcpy(p, &f16, 2U); p += 2U;

    // float32[3] accelerometer_integral -- zeroed
    p += 12U;

    // float16[<=36] covariance -- TAO tail array, 0 items, nothing to encode

    return DRONE_CAN_DSDL_AHRS_RAWIMU_PAYLOAD_SIZE;
}

/// @brief Serialize a uavcan.equipment.air_data.StaticPressure payload.
///
/// @param [out] apBuf              Output buffer, at least
///                                 DRONE_CAN_DSDL_AIR_DATA_STATIC_PRESSURE_PAYLOAD_SIZE bytes.
/// @param [in]  fPressurePa        Static pressure in Pascals.
/// @param [in]  fVariancePa2       Pressure variance in Pa^2 (set to 0 if unknown).
/// @return                         Payload length in bytes.
static inline uint16_t DroneCanDsdl_SerializeStaticPressure(
    uint8_t* apBuf,
    float fPressurePa,
    float fVariancePa2)
{
    uint16_t f16;
    uint8_t* p = apBuf;

    // float32 static_pressure
    memcpy(p, &fPressurePa, 4U); p += 4U;

    // float16 static_pressure_variance
    f16 = canardConvertNativeFloatToFloat16(fVariancePa2);
    memcpy(p, &f16, 2U);

    return DRONE_CAN_DSDL_AIR_DATA_STATIC_PRESSURE_PAYLOAD_SIZE;
}

/// @brief Serialize a uavcan.equipment.air_data.StaticTemperature payload.
///
/// @param [out] apBuf              Output buffer, at least
///                                 DRONE_CAN_DSDL_AIR_DATA_STATIC_TEMPERATURE_PAYLOAD_SIZE bytes.
/// @param [in]  fTemperatureK      Static temperature in Kelvin.
/// @param [in]  fVarianceK2        Temperature variance in K^2 (set to 0 if unknown).
/// @return                         Payload length in bytes.
static inline uint16_t DroneCanDsdl_SerializeStaticTemperature(
    uint8_t* apBuf,
    float fTemperatureK,
    float fVarianceK2)
{
    uint16_t f16;
    uint8_t* p = apBuf;

    // float16 static_temperature
    f16 = canardConvertNativeFloatToFloat16(fTemperatureK);
    memcpy(p, &f16, 2U); p += 2U;

    // float16 static_temperature_variance
    f16 = canardConvertNativeFloatToFloat16(fVarianceK2);
    memcpy(p, &f16, 2U);

    return DRONE_CAN_DSDL_AIR_DATA_STATIC_TEMPERATURE_PAYLOAD_SIZE;
}

#ifdef __cplusplus
}
#endif

#endif // DRONE_CAN_DSDL_MESSAGES_H
