\page page_px4_integration PX4 integration

This chapter describes how to integrate the BAHRS with a PX4 flight controller.

The device offers two output interfaces towards a flight controller: a serial link, for which
the upstream PX4 source tree provides a driver, and DroneCAN, which is provided as a separate
BAHRS firmware build variant.

The two interfaces are **not alternatives**. The serial output is active in every firmware
variant, so a device built for DroneCAN transmits on both interfaces at the same time. BAHRS
transmits its attitude and heading solution over both interfaces. A PX4 firmware build containing
the `eulernav_bahrs` driver consumes the serial solution, but upstream PX4 has no bridge that
converts the DroneCAN `uavcan.equipment.ahrs.Solution` message into the `vehicle_attitude` topic.
See @ref subsec_px4_dronecan_consumption and @ref subsec_px4_choosing.

@section sec_px4_overview Overview

Two BAHRS hardware interface variants are available. They use the same serial protocol and the
same PX4 `eulernav_bahrs` driver, but their electrical interfaces are different:

- **BAHRS interface Variant 1** provides an RS232 interface. An RS232-to-3.3 V TTL converter is
  required between the BAHRS and a Pixhawk UART.
- **BAHRS interface Variant 2** provides a 3.3 V TTL UART interface and can be connected directly
  to a compatible 3.3 V Pixhawk UART.

@subsection subsec_px4_overview_variant1 Serial connection for BAHRS interface Variant 1

@startuml{px4_integration_variant1.png} "BAHRS Variant 1 interfaces towards a PX4 flight controller" width=16cm

left to right direction

[BAHRS Variant 1] as Bahrs
[RS232 to 3.3 V TTL converter] as Converter
[PX4 flight controller] as Px4

interface "RS232" as SerialRs232
interface "3.3 V UART" as SerialTtl
interface "DroneCAN (CAN)" as DroneCan

Bahrs -- SerialRs232
SerialRs232 )-- Converter
Converter -- SerialTtl
SerialTtl )-- Px4

Bahrs -- DroneCan
DroneCan )-- Px4

@enduml

@warning Do not connect the RS232 signals of BAHRS interface Variant 1 directly to a Pixhawk
UART. Their electrical voltage levels are not compatible. Use a suitable bidirectional
RS232-to-3.3 V TTL converter.

@subsection subsec_px4_overview_variant2 Serial connection for BAHRS interface Variant 2

@startuml{px4_integration_variant2.png} "BAHRS Variant 2 interfaces towards a PX4 flight controller" width=14cm

left to right direction

[BAHRS Variant 2] as Bahrs
[PX4 flight controller] as Px4

interface "3.3 V UART" as Serial
interface "DroneCAN (CAN)" as DroneCan

Bahrs -- Serial
Serial )-- Px4

Bahrs -- DroneCan
DroneCan )-- Px4

@enduml

No external level converter is required for interface Variant 2 when the selected Pixhawk port
uses compatible 3.3 V UART levels.

@subsection subsec_px4_overview_interfaces Available data paths

The serial output remains active in firmware built with `USE_DRONECAN`. Serial and DroneCAN can
therefore operate at the same time; they are not mutually exclusive.

| Property | Serial, interface Variant 1 | Serial, interface Variant 2 | DroneCAN |
| --- | --- | --- | --- |
| Physical interface | RS232 | 3.3 V TTL UART | CAN |
| Connection to Pixhawk | Through RS232-to-3.3 V TTL converter | Direct UART connection | CAN high, CAN low, and ground |
| BAHRS firmware | Available in all builds | Available in all builds | `USE_DRONECAN` build required |
| PX4 software | `eulernav_bahrs` driver, must be enabled in the PX4 build | `eulernav_bahrs` driver, must be enabled in the PX4 build | `uavcan` module and message-specific bridges |
| BAHRS transmits attitude and heading solution | Yes | Yes | Yes, as `uavcan.equipment.ahrs.Solution` |
| PX4 publishes the BAHRS solution as `vehicle_attitude` | Yes, if the serial driver is included | Yes, if the serial driver is included | **No**, a dedicated PX4 bridge is missing |
| Raw inertial data | Yes | Yes | Yes |
| Barometric data | Yes | Yes | Yes |

@note BAHRS broadcasts its attitude and heading solution over DroneCAN. The corresponding
`uavcan.equipment.ahrs.Solution` frames are present on the CAN bus and can be captured and
decoded. However, upstream PX4 does not currently contain a bridge that subscribes to this
message and publishes the solution as `vehicle_attitude`. This is a limitation of the PX4
integration, not of the BAHRS DroneCAN output.

@section sec_px4_serial Serial integration

This is the fully featured integration path supported by the upstream `eulernav_bahrs` driver. It
is currently the only path through which PX4 publishes the BAHRS attitude and heading solution as
`vehicle_attitude`.

@subsection subsec_px4_serial_link Physical link

The serial output is implemented by @link CRs232OutputHandler @endlink.
The required physical connection depends on the BAHRS hardware interface variant, as shown in
@ref subsec_px4_overview_variant1 and @ref subsec_px4_overview_variant2.

| Parameter | Value |
| --- | --- |
| Interface | USART1 |
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |

@note Firmware built with `SEND_DEBUG_OUTPUT` reconfigures USART1 to 921600 baud in order to
carry additional debug output. Such builds are not compatible with the PX4 driver's expected
baud rate. Use a production build when connecting to a flight controller.

@subsection subsec_px4_serial_messages Transmitted messages

The message set is defined by `CSerialProtocol::EMessageIds`.

| ID | Message |
| --- | --- |
| 0x01 | Inertial data |
| 0x02 | Navigation data |
| 0x03 | Attitude accuracy information |
| 0x04 | Time of navigation data |
| 0x05 | Time of inertial data |
| 0x06 | Time of the latest sync pulse |

@subsection subsec_px4_serial_px4_side PX4 configuration

The upstream PX4 source tree contains a serial bus driver for the device, named
`eulernav_bahrs`. To conserve flash memory, external INS and AHRS drivers are not included in
default PX4 board configurations. Before building PX4 firmware, enable
`CONFIG_DRIVERS_INS_EULERNAV_BAHRS=y` in the configuration for the target flight controller.

After flashing a PX4 firmware image containing the driver, connect the BAHRS to a free serial
port of the flight controller using the wiring required by its interface variant, and start the
driver on that port:

    eulernav_bahrs start -d /dev/ttyS1

Further commands are:

    eulernav_bahrs status
    eulernav_bahrs stop

The `-d` argument selects the serial device; replace `/dev/ttyS1` with the port the BAHRS is
actually connected to on the target board.

@section sec_px4_dronecan DroneCAN integration

DroneCAN (UAVCAN v0) support is implemented by @link CDroneCanHandler @endlink and
@link CDroneCanOutputHandler @endlink.

@subsection subsec_px4_dronecan_variant Firmware build variant

DroneCAN is a compile-time build variant selected by the `USE_DRONECAN` macro. The default build
keeps the legacy CANaerospace stack, so a dedicated firmware image has to be built for DroneCAN
operation.

| Target | How to build the DroneCAN variant |
| --- | --- |
| BAHRS HW V2 | Build configuration `Release_DroneCAN` |
| BAHRS HW V3 | Build configuration `Debug_DroneCAN` or `Release_DroneCAN`, or the CMake option `-DUSE_DRONECAN=ON` |

@subsection subsec_px4_dronecan_rte Runtime environment of the variant

Both CAN stacks now share one runtime environment configuration per hardware target. A compile-time
alias header maps the canonical CAN component names (`CCanHandler`, `CCanOutputHandler`) to either
the CANaerospace or DroneCAN implementation, depending on `USE_DRONECAN`.

To verify which stack ended up in an image, check its symbols - exactly one of the two must be
present:

    arm-none-eabi-nm <image>.elf | findstr /C:"CDroneCan"
    arm-none-eabi-nm <image>.elf | findstr /C:"CCanHandler"

@subsection subsec_px4_dronecan_bus Bus parameters

| Parameter | Value |
| --- | --- |
| Bit rate | 500 kbit/s |
| Frame format | Extended (29 bit identifiers) |
| CAN peripheral, HW V2 | CAN2 |
| CAN peripheral, HW V3 | CAN1 |

The CAN peripheral is selected at build time by `CAN_INTERFACE_NUMBER`, which defaults to 2
(CAN2) and is set to 1 for HW V3.

@note The bit rate is fixed at 500 kbit/s in the firmware, whereas the PX4 default is 1 Mbit/s.
The flight controller must be configured to 500 kbit/s, otherwise the mismatch floods the bus
with error frames and no node is discovered.

Exactly two nodes at the physical ends of the bus must terminate it with 120 Ohm. Verify the
termination of the whole harness rather than of a single device, because flight controllers and
CAN adapters often provide a switchable or permanently fitted terminator of their own.

@subsection subsec_px4_dronecan_node Node identity

| Property | Value |
| --- | --- |
| Node ID | 45 by default |
| Node name | `ams.bahrs` |
| `uavcan.protocol.NodeStatus` | Broadcast at 1 Hz |
| `uavcan.protocol.GetNodeInfo` | Answered with node name, software and hardware version, and the unique 96 bit MCU identifier |

The node ID can be overridden through the CAN configuration block in non-volatile memory. The
stored value is only applied if its CRC is valid, otherwise the default is used.

@note A node that does not broadcast `uavcan.protocol.NodeStatus` is not discovered by PX4 or by
DroneCAN inspection tools, even though it may be transmitting sensor messages correctly.

@subsection subsec_px4_dronecan_messages Broadcast messages

| Message | Data type ID | Rate | Source inside the device |
| --- | --- | --- | --- |
| `uavcan.protocol.NodeStatus` | 341 | 1 Hz | Uptime, health, and mode of the node |
| `uavcan.equipment.ahrs.RawIMU` | 1003 | 100 Hz | Monitored IMU output |
| `uavcan.equipment.ahrs.Solution` | 1000 | 100 Hz | Safety-monitored roll and pitch, safety-monitored magnetic heading, angular rate and specific force from the monitored IMU output |
| `uavcan.equipment.air_data.StaticPressure` | 1028 | 25 Hz | First available of the three voted barometric pressure channels |
| `uavcan.equipment.air_data.StaticTemperature` | 1029 | 25 Hz | First available of the three voted barometric temperature channels |

The orientation reported in `uavcan.equipment.ahrs.Solution` is deliberately taken from the
safety-monitored attitude and the safety-monitored magnetic heading (the output of the magnetic
heading monitor), and not from a single unmonitored filter instance, which mirrors the behaviour
of the CANaerospace output. The quaternion is
assembled from roll, pitch, and magnetic heading, and is converted from the internal
[W, X, Y, Z] convention to the [X, Y, Z, W] field order used by DroneCAN.

The message is suppressed unless all of the following hold: attitude, heading, and inertial data
are readable; the attitude health is not reported as unavailable; the magnetic heading health is
not reported as unavailable (a heading with health `integrity risk` is still sent); the inertial signals are not reported as unavailable; and roll, pitch, heading, specific
force, and angular rate all pass their plausibility range checks. A silent absence of this
message therefore indicates a data quality problem rather than a bus problem.

@subsection subsec_px4_dronecan_consumption What PX4 actually consumes

PX4 implements a fixed set of DroneCAN sensor bridges. Each broadcast message is only used if a
matching bridge exists.

| Message broadcast by the BAHRS | Frames present on CAN bus | Consumed by upstream PX4 | PX4 bridge |
| --- | --- | --- | --- |
| `uavcan.protocol.NodeStatus` | Yes | Yes | Node discovery and health monitoring |
| `uavcan.equipment.ahrs.RawIMU` | Yes | Yes | Accelerometer and gyroscope bridges |
| `uavcan.equipment.air_data.StaticPressure` | Yes | Yes | Barometer bridge |
| `uavcan.equipment.air_data.StaticTemperature` | Yes | Yes | Barometer bridge |
| `uavcan.equipment.ahrs.Solution` | Yes | **No** | No AHRS bridge exists in PX4 |

@warning BAHRS broadcasts `uavcan.equipment.ahrs.Solution`, and these frames are present on the
CAN bus and can be captured and decoded. Upstream PX4 does not subscribe to or translate this
message, so it does not publish the BAHRS solution as `vehicle_attitude`. Using the solution over
DroneCAN requires an additional bridge on the PX4 side.

@subsection subsec_px4_dronecan_solution_options Options for delivering the attitude solution

No change on the DroneCAN side alone can make upstream PX4 use the attitude solution, because PX4
does not subscribe to an attitude solution message over CAN. Defining a vendor specific data type
does not help either, since it would still need a subscriber on the flight controller. Three
options exist.

-# **Serial for the solution, DroneCAN for the sensors.** Requires no changes at all and works
   with the current firmware, because the serial output is active in the DroneCAN variant as
   well. The flight controller takes the attitude and heading solution through the
   `eulernav_bahrs` driver, while the DroneCAN node contributes inertial and barometric data,
   health, and identification on the shared harness. Verify timing with both outputs active.
-# **DroneCAN only, with the estimate computed by PX4.** Requires no changes to PX4, but the
   device would additionally have to publish `uavcan.equipment.ahrs.MagneticFieldStrength` or
   `uavcan.equipment.ahrs.MagneticFieldStrength2`, which the PX4 magnetometer bridge accepts, so
   that the flight controller has a complete sensor set. The attitude is then estimated by PX4
   and is **not** the solution of the BAHRS, which means the redundancy and the monitoring of
   the device no longer contribute to it.
-# **A new sensor bridge in PX4.** The only option that delivers the solution of the device over
   CAN. A bridge subscribing to `uavcan.equipment.ahrs.Solution` republishes it as an external
   attitude estimate on the `vehicle_visual_odometry` topic, which the estimator fuses once the
   corresponding bit of `EKF2_EV_CTRL` is enabled. Publishing `vehicle_attitude` directly and
   bypassing the estimator is possible but contradicts the architecture of PX4 and is not
   recommended. As PX4 already contains a serial driver for the device, such a bridge is a
   plausible upstream contribution and should be discussed with the PX4 maintainers rather than
   maintained as a private fork.

@subsection subsec_px4_dronecan_px4_side PX4 configuration

| Parameter | Purpose |
| --- | --- |
| `UAVCAN_ENABLE` | 0 disables DroneCAN; 1 enables it for sensors; 2 additionally enables the dynamic node allocation server; 3 enables sensors and ESCs |
| `UAVCAN_BITRATE` | Bus bit rate, must be set to 500000 |
| `UAVCAN_NODE_ID` | Node ID of the flight controller itself, 1 by default |
| `UAVCAN_SUB_IMU` | Enables the accelerometer and gyroscope bridges |
| `UAVCAN_SUB_BARO` | Enables the barometer bridge |
| `CANNODE_TERM` | Enables bus termination on the flight controller, where supported |

@note DroneCAN parameter changes only take effect after the flight controller has been rebooted.
Changing the bit rate without a reboot is a frequent cause of a node not appearing.

@subsection subsec_px4_choosing Choosing an interface

Because the serial output is present in every firmware variant, the question is not which
interface to build, but which interface the flight controller is wired to and configured for.

Use the **serial** interface whenever PX4 shall use the attitude and heading solution of the
BAHRS. This is what the device is designed to provide, and the driver is part of the upstream PX4
source tree. The driver must be enabled in the PX4 build because it is not included in default
board configurations. Use an RS232-to-3.3 V TTL converter with BAHRS interface Variant 1; connect
interface Variant 2 directly to a compatible 3.3 V Pixhawk UART.

Add **DroneCAN** when the device shall additionally appear as a node on an existing CAN bus, for
inertial and barometric data, node health, and identification over a single shared harness.

Use **DroneCAN alone** only if it is acceptable that the flight controller estimates attitude
itself from the transmitted sensor data, or if one of the extensions described in
@ref subsec_px4_dronecan_solution_options is implemented.

@section sec_px4_bringup Bring-up and verification

@subsection subsec_px4_bringup_serial Serial interface

-# Flash a production firmware image, that is, one built without `SEND_DEBUG_OUTPUT`.
-# Build and flash PX4 firmware with `CONFIG_DRIVERS_INS_EULERNAV_BAHRS=y` enabled for the target
   flight controller. The driver is not included in default PX4 board configurations.
-# Identify whether the device is BAHRS interface Variant 1 (RS232) or Variant 2 (3.3 V UART).
-# For Variant 1, install a bidirectional RS232-to-3.3 V TTL converter between the BAHRS and the
   Pixhawk UART. For Variant 2, connect the BAHRS directly to a compatible 3.3 V Pixhawk UART.
-# Cross TX and RX, and connect signal ground.
-# Start the driver with `eulernav_bahrs start -d <device>`.
-# Confirm operation with `eulernav_bahrs status`.

@subsection subsec_px4_bringup_dronecan DroneCAN interface

-# Flash a firmware image built with the `USE_DRONECAN` variant and confirm the image contains
   the DroneCAN symbols.
-# Wire CAN high and CAN low, and check that the harness is terminated at both ends.
-# Set `UAVCAN_ENABLE` and `UAVCAN_BITRATE` to 500000 on the flight controller, then reboot it.
-# Enable `UAVCAN_SUB_IMU` and `UAVCAN_SUB_BARO`.
-# Verify that the node appears with node ID 45 and name `ams.bahrs`.
-# Verify that inertial and barometric measurements arrive in PX4.

For inspection independent of a flight controller, the repository provides an SLCAN based
monitor at `06_Tools/DroneCAN/dronecan_diag.py`. Third party tools such as the DroneCAN GUI tool
can also be used to enumerate the node and to decode its messages.

@section sec_px4_troubleshooting Troubleshooting

| Symptom | Likely cause |
| --- | --- |
| The node does not appear at all | Bit rate mismatch, flight controller not rebooted after a parameter change, missing or duplicated bus termination, or CAN high and CAN low swapped |
| The node appears but has no name | `uavcan.protocol.GetNodeInfo` is not answered, which indicates that reception is not working while transmission is |
| The node is listed but no measurements arrive | The corresponding `UAVCAN_SUB_*` subscription is disabled |
| DroneCAN `Solution` frames are visible, but `vehicle_attitude` is not updated from them | Upstream PX4 has no bridge for `uavcan.equipment.ahrs.Solution`, see @ref subsec_px4_dronecan_consumption |
| `eulernav_bahrs` is not recognized by the PX4 shell | The driver was not enabled in the PX4 build; rebuild PX4 with `CONFIG_DRIVERS_INS_EULERNAV_BAHRS=y` |
| `uavcan.equipment.ahrs.Solution` is missing although the bus is healthy | The message is suppressed because a signal is unavailable, the magnetic heading is unavailable, or a value failed its range check |
| The DroneCAN variant behaves as if an RTE change had no effect | The target RTE was not regenerated after changing `rte.json`, see @ref subsec_px4_dronecan_rte |
| Frames are lost when using an SLCAN adapter | The serial link of the adapter, and not the CAN bus, is the bottleneck at the full broadcast rates |
| Two nodes conflict on the bus | The BAHRS node ID collides with another node; change it through the CAN configuration block in non-volatile memory |
