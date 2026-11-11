/*
 * CANaerospace message set extension for small UAV systems
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_PARAM_UAV_H_
#define CANAEROSPACE_PARAM_UAV_H_

/*
 * Allowed ranges:
 *   CANAS_MSGTYPE_USER_DEFINED_HIGH_MIN = 200
 *   CANAS_MSGTYPE_USER_DEFINED_HIGH_MAX = 299
 * and
 *   CANAS_MSGTYPE_USER_DEFINED_LOW_MIN = 1800
 *   CANAS_MSGTYPE_USER_DEFINED_LOW_MAX = 1899
 */

typedef enum
{
    /**
     * ESC control
     * Type: USHORT
     * Units: ESC thrust normalized into 1..65535; 0 - disarm
     */
    CANAS_UAV_ESC_COMMAND_1 = 200,
    CANAS_UAV_ESC_COMMAND_2,
    CANAS_UAV_ESC_COMMAND_3,
    CANAS_UAV_ESC_COMMAND_4,
    CANAS_UAV_ESC_COMMAND_5,
    CANAS_UAV_ESC_COMMAND_6,
    CANAS_UAV_ESC_COMMAND_7,
    CANAS_UAV_ESC_COMMAND_8
} CanasUavMsgIdHigh;

typedef enum
{
    /**
     * Rotor RPM
     * Type: USHORT
     * Units: RPM
     */
    CANAS_UAV_ROTOR_RPM_1 = 1800,
    CANAS_UAV_ROTOR_RPM_2,
    CANAS_UAV_ROTOR_RPM_3,
    CANAS_UAV_ROTOR_RPM_4,
    CANAS_UAV_ROTOR_RPM_5,
    CANAS_UAV_ROTOR_RPM_6,
    CANAS_UAV_ROTOR_RPM_7,
    CANAS_UAV_ROTOR_RPM_8,

    /**
     * Gimbal angle or angular rate, respectively
     * Type: FLOAT
     * Units: deg or deg/sec, respectively
     */
    CANAS_UAV_GIMBAL_ROLL = 1810,
    CANAS_UAV_GIMBAL_ROLL_RATE,
    CANAS_UAV_GIMBAL_PITCH,
    CANAS_UAV_GIMBAL_PITCH_RATE,
    CANAS_UAV_GIMBAL_YAW,
    CANAS_UAV_GIMBAL_YAW_RATE,

    /**
     * Gripper control
     * Type: CHAR
     * Values: 0 - free, 1..255 - grasp
     */
    CANAS_UAV_GRIPPER_GRIP_COMMAND = 1820,
    CANAS_UAV_GRIPPER_GRIP_STATE           ///< Feedback
} CanasUavMsgIdLow;

#endif
