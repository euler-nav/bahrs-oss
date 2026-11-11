\page page_building_block_view Building block view

@section sec_bb_view_level_0 Level 0

@startuml{bahrs_sw_class_diagram_level_0.png} "BAHRS SW class diagram" width=10cm

class "BAHRS Software" as BahrsSw
class "Bootloader" as Bootloader
class "Target Application" as TargetApp

BahrsSw "1" o-- "1" Bootloader
BahrsSw "1" o-- "1" TargetApp

@enduml

@section sec_bb_view_level_1 Level 1

Target Application is devided into Application Software (ASW), Basic Software (BSW), Runtime Environment (RTE), and library collection.

@startuml{bahrs_sw_class_diagram.png} "BAHRS Target Application class diagram" width=10cm
class "Target Application" as TargetApp
class "Application Software" as asw
class "Basic Software" as bsw
class "Runtime Environment" as rte
class "Library Collection" as LibraryCollection

TargetApp "1" o-- "1" asw
TargetApp "1" o-- "1" bsw
TargetApp "1" o-- "1" rte
TargetApp "1" o-- "1" LibraryCollection

@enduml

@subsection subsec_asw Application software

ASW is a collection of software components that enable the device to perform its main functions.

@subsection subsec_bsw Basic software

BSW is a collection of software components and libraries that implement generic embedded functionality, such as initialization of peripherals, scheduling, output data transmission, etc.

@subsection subsec_rte Runtime Environment

RTE is a middleware that enable safe data exchange between software components. RTE presents a collection of software component ports that are used by software components for data exchange.

@section sec_bb_view_level_2 Level 2

@subsection subsec_bsw_level_2 Basic software

@startuml{bb_view_level_2_bsw.png} "Basic Software Class Diagram" width=15cm
"Basic Software" "1" o-- "1" RTOS
"Basic Software" "1" o-- "1" "STM32 Hardware Abstraction Layer"
"Basic Software" "1" o-- "*" "Base Software Component"
RTOS "1" o-- "1" "CMSIS-OS"
RTOS "1" o-- "1" FreeRTOS
@enduml

Unlike application software components, basic software components can use HAL APIs.

@subsubsection subsubsec_bsw_swc_list List of basic software components

-# BMP384 sensor driver: implemented by @link CBmp384Driver @endlink
-# BMM150 sensor 1 driver: implemented by @link CBmm150Driver @endlink
-# BMM150 sensor 2 driver: implemented by @link CBmm150Driver @endlink
-# SCHA63T sensor driver: implemented by @link CScha63TDriver @endlink
-# ICM20789 sensor 1 driver: implemented by @link CIcm20789Driver @endlink
-# ICM20789 sensor 2 driver: implemented by @link CIcm20789Driver @endlink
-# MMC5983 sensor driver: implemented by @link CMmc5983Driver @endlink
-# RS232 output handler: implemented by @link CRs232OutputHandler @endlink
-# CAN handler: implemented by @link CCanHandler @endlink
-# CAN output handler: implemented by @link CCanOutputHandler @endlink
-# Sync pulse handler: implemented by @link CSyncPulseHandler @endlink

@subsection subsec_rte_level_2 Runtime environment

Runtime environment (RTE) is a collection of software component (SWC) ports and their attributes. RTE is responsible for initialization of SWC ports. A port is a mutex-protected memory that is used by software components for data exchange. Software component ports are implemented by the template CSoftwareComponentPort. RTE is implemented as a singleton object (CRte) with static memory allocation. SWC ports are private members of CRte. Only selected friend functions and classes are allowed to access SWC ports.

@startuml{bb_view_level_2_rte.png} "CRte class diagram"
CRte "1" *-- "*" "Software Component Port"

class CRte {
+GetInstance()
}

class "Software Component Port" {
+Init()
+Read()
+Write()
}
@enduml

@subsection subsec_asw_level_2 Application software

Application software is comprised of multiple software components that implement core functionality of the device. Application software components must not use HAL APIs directly.

@subsubsection subsubsec_asw_swc_list List of application software components

-# IMU pre-filter 1: implemented by @link CImuPreFilterSwc @endlink
-# IMU pre-filter 2: implemented by @link CImuPreFilterSwc @endlink
-# IMU pre-filter 3: implemented by @link CImuPreFilterSwc @endlink
-# Pressure compensator 1: implemented by @link CPressureCompensator @endlink
-# Pressure compensator 2: implemented by @link CPressureCompensator @endlink
-# Pressure compensator 3: implemented by @link CPressureCompensator @endlink
-# IMU Monitor: implemented by @link CImuMonitorSwc @endlink
-# Barometer monitor: implemented by @link CBaroMonitorSwc @endlink
-# Magnetometer compensator 1: implemented by @link CMagnetometerCompensator @endlink
-# Magnetometer compensator 2: implemented by @link CMagnetometerCompensator @endlink
-# Magnetometer compensator 3: implemented by @link CMagnetometerCompensator @endlink
-# Output transformer: implemented by @link COutputTransformer @endlink
-# BAHRS filter 1: implemented by @link CBahrsFilterSwc @endlink
-# BAHRS filter 2: implemented by @link CBahrsFilterSwc @endlink
-# BAHRS filter 3: implemented by @link CBahrsFilterSwc @endlink
-# Attitude monitor: implemented by @link CAttitudeMonitorSwc @endlink
-# Vertical channel monitor: implemented by @link CVerticalChannelMonitorSwc @endlink
-# Magnetic heading filter 1: implemented by @link CMagneticHeadingFilterSwc @endlink
-# Magnetic heading filter 2: implemented by @link CMagneticHeadingFilterSwc @endlink
-# Magnetic heading filter 3: implemented by @link CMagneticHeadingFilterSwc @endlink
-# Magnetic heading monitor: implemented by @link CMagneticHeadingMonitorSwc @endlink

@section sec_bb_view_level_3 Level 3

@subsection subsec_asw_level_3 Application software

@subsubsection subsubsec_asw_level_3_component_diagram UML component diagram

@startuml{asw_component_diagram.png} "Component diagram: application software"

left to right direction

interface PressureDataSensor1
interface PressureDataSensor2
interface PressureDataSensor3
interface CompensatedPressureDataSensor1
interface CompensatedPressureDataSensor2
interface CompensatedPressureDataSensor3
interface ImuDataSensor1
interface ImuDataSensor2
interface ImuDataSensor3
interface ImuDataFilteredSensor1
interface ImuDataFilteredSensor2
interface ImuDataFilteredSensor3
interface MagneticFieldSensor1
interface MagneticFieldSensor2
interface MagneticFieldSensor3

[Pressure Compensator 1] as PressureCompensator1
[Pressure Compensator 2] as PressureCompensator2
[Pressure Compensator 3] as PressureCompensator3

PressureDataSensor1 )-- PressureCompensator1
PressureDataSensor2 )-- PressureCompensator2
PressureDataSensor3 )-- PressureCompensator3

[IMU Pre-filter 1] as ImuPreFilter1
[IMU Pre-filter 2] as ImuPreFilter2
[IMU Pre-filter 3] as ImuPreFilter3

ImuDataSensor1 )-- ImuPreFilter1
ImuDataSensor2 )-- ImuPreFilter2
ImuDataSensor3 )-- ImuPreFilter3

PressureCompensator1 -- CompensatedPressureDataSensor1
PressureCompensator2 -- CompensatedPressureDataSensor2
PressureCompensator3 -- CompensatedPressureDataSensor3

ImuPreFilter1 -- ImuDataFilteredSensor1
ImuPreFilter2 -- ImuDataFilteredSensor2
ImuPreFilter3 -- ImuDataFilteredSensor3

[Barometer Monitor] as BaroMonitor

CompensatedPressureDataSensor1 )-- BaroMonitor
CompensatedPressureDataSensor2 )-- BaroMonitor
CompensatedPressureDataSensor3 )-- BaroMonitor

interface SafePressureData1
interface SafePressureData2
interface SafePressureData3

[Magnetometer Compensator 1] as MagnetometerCompensator1
[Magnetometer Compensator 2] as MagnetometerCompensator2
[Magnetometer Compensator 3] as MagnetometerCompensator3
MagneticFieldSensor1 )-- MagnetometerCompensator1
MagneticFieldSensor2 )-- MagnetometerCompensator2
MagneticFieldSensor3 )-- MagnetometerCompensator3

interface CompensatedMagneticFieldData1
interface CompensatedMagneticFieldData2
interface CompensatedMagneticFieldData3
MagnetometerCompensator1 -- CompensatedMagneticFieldData1
MagnetometerCompensator2 -- CompensatedMagneticFieldData2
MagnetometerCompensator3 -- CompensatedMagneticFieldData3

[BAHRS Filter 1] as BahrsFilter1
[BAHRS Filter 2] as BahrsFilter2
[BAHRS Filter 3] as BahrsFilter3

ImuDataFilteredSensor1 )-- BahrsFilter1
ImuDataFilteredSensor2 )-- BahrsFilter2
ImuDataFilteredSensor3 )-- BahrsFilter3

interface BahrsFilterOutput1
interface BahrsFilterOutput2
interface BahrsFilterOutput3

SafePressureData1 )-- BahrsFilter1
BahrsFilter1 -- BahrsFilterOutput1

SafePressureData2 )-- BahrsFilter2
BahrsFilter2 -- BahrsFilterOutput2

SafePressureData3 )-- BahrsFilter3
BahrsFilter3 -- BahrsFilterOutput3

[IMU monitor] as ImuMonitor
ImuDataFilteredSensor1 )-- ImuMonitor
ImuDataFilteredSensor2 )-- ImuMonitor
ImuDataFilteredSensor3 )-- ImuMonitor

interface SafeImuData
ImuMonitor -- SafeImuData

BaroMonitor -- SafePressureData1
BaroMonitor -- SafePressureData2
BaroMonitor -- SafePressureData3

[Output Transformer] as OutputTransformer

interface VehicleAttitude1
interface VehicleAttitude2
interface VehicleAttitude3
interface MagneticFieldDataInVehicleFrame1
interface MagneticFieldDataInVehicleFrame2
interface MagneticFieldDataInVehicleFrame3
interface SafeImuDataInVehicleFrame

BahrsFilterOutput1 )-- OutputTransformer
BahrsFilterOutput2 )-- OutputTransformer
BahrsFilterOutput3 )-- OutputTransformer
CompensatedMagneticFieldData1 )-- OutputTransformer
CompensatedMagneticFieldData2 )-- OutputTransformer
CompensatedMagneticFieldData3 )-- OutputTransformer
SafeImuData )-- OutputTransformer
OutputTransformer -- VehicleAttitude1
OutputTransformer -- VehicleAttitude2
OutputTransformer -- VehicleAttitude3
OutputTransformer -- MagneticFieldDataInVehicleFrame1
OutputTransformer -- MagneticFieldDataInVehicleFrame2
OutputTransformer -- MagneticFieldDataInVehicleFrame3
OutputTransformer -- SafeImuDataInVehicleFrame

[Attitude Monitor] as AttitudeMonitor
VehicleAttitude1 )-- AttitudeMonitor
VehicleAttitude2 )-- AttitudeMonitor
VehicleAttitude3 )-- AttitudeMonitor

[Vertical Channel Monior] as VerticalChannelMonior
BahrsFilterOutput1 )-- VerticalChannelMonior
BahrsFilterOutput2 )-- VerticalChannelMonior
BahrsFilterOutput3 )-- VerticalChannelMonior

interface SafeVehicleAttitude
interface SafeVerticalChannelData

AttitudeMonitor -- SafeVehicleAttitude
VerticalChannelMonior -- SafeVerticalChannelData

[Magnetic Heading Filter 1] as MagneticHeadingFilter1
[Magnetic Heading Filter 2] as MagneticHeadingFilter2
[Magnetic Heading Filter 3] as MagneticHeadingFilter3

SafeVehicleAttitude )-- MagneticHeadingFilter1
SafeVehicleAttitude )-- MagneticHeadingFilter2
SafeVehicleAttitude )-- MagneticHeadingFilter3
MagneticFieldDataInVehicleFrame1 )-- MagneticHeadingFilter1
MagneticFieldDataInVehicleFrame2 )-- MagneticHeadingFilter2
MagneticFieldDataInVehicleFrame3 )-- MagneticHeadingFilter3

interface MagneticHeading1
interface MagneticHeading2
interface MagneticHeading3

MagneticHeadingFilter1 -- MagneticHeading1
MagneticHeadingFilter2 -- MagneticHeading2
MagneticHeadingFilter3 -- MagneticHeading3

[Magnetic Heading Monitor] as MagneticHeadingMonitor

MagneticHeading1 )-- MagneticHeadingMonitor
MagneticHeading2 )-- MagneticHeadingMonitor
MagneticHeading3 )-- MagneticHeadingMonitor

interface SafeMagneticHeading

MagneticHeadingMonitor -- SafeMagneticHeading

@enduml

@subsection subsec_bsw_level_3 Basic software

@subsubsection subsubsec_bsw_level_3_component_diagram UML component diagram

@startuml{bsw_component_diagram.png} "Component diagram: basic software"

left to right direction

[BMP384 sensor driver] as Bmp384Driver
Bmp384Driver -- PressureDataSensor1

[BMM150 sensor 1 driver] as Bmm150Driver1
Bmm150Driver1 -- MagneticFieldSensor2

[BMM150 sensor 2 driver] as Bmm150Driver2
Bmm150Driver2 -- MagneticFieldSensor3

[SCHA63T sensor driver] as Scha63TDriver
Scha63TDriver -- ImuDataSensor1

[ICM20789 sensor 1 driver] as Icm20789Driver1
Icm20789Driver1 -- ImuDataSensor2
Icm20789Driver1 -- PressureDataSensor2

[ICM20789 sensor 2 driver] as Icm20789Driver2
Icm20789Driver2 -- ImuDataSensor3
Icm20789Driver2 -- PressureDataSensor3

[MMC5983 sensor driver] as Mmc5983Driver
Mmc5983Driver -- MagneticFieldSensor1

[Sync pulse handler] as SyncPulseHandler
SyncPulseHandler -- TimeOfLatestSyncPulse

[RS232 output handler] as Rs232OutputHanler
SafeVehicleAttitude )-- Rs232OutputHanler
SafeVerticalChannelData )-- Rs232OutputHanler
SafeImuDataInVehicleFrame )-- Rs232OutputHanler
SafeMagneticHeading )-- Rs232OutputHanler
TimeOfLatestSyncPulse )-- Rs232OutputHanler

[CAN handler] as CanHandler

[CAN output handler] as CanOutputHandler
SafeVehicleAttitude )-- CanOutputHandler
SafeVerticalChannelData )-- CanOutputHandler
SafeImuDataInVehicleFrame )-- CanOutputHandler
SafeMagneticHeading )-- CanOutputHandler
SafePressureData1 )-- CanOutputHandler
SafePressureData2 )-- CanOutputHandler
SafePressureData3 )-- CanOutputHandler


@enduml
