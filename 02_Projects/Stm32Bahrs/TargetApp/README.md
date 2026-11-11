# Introduction

The folder contains source code of the firmware of EULER-NAV Baro-Inertial AHRS, BAHRS rev. 2.

The firmware is built with CMake and Ninja, the same way as BAHRS rev. 3 (see *..\TargetApp_HW_V3\README.md* for the toolchain installation).
To work in STM32CubeIDE, import this folder (*TargetApp*) with the same steps as described for BAHRS rev. 3. Its *.project* and *.cproject* use the CMake build.

# How to build the firmware

From the repository root run

```
05_Jenkins\Build_TargetApp_STM32_V2.bat <debug|release|debug-with-debug-output> [dronecan]
```

The optional second argument `dronecan` builds the DroneCAN CAN stack instead of CANaerospace (use with `release`).
The variant argument is required and must be lowercase. `debug-with-debug-output` builds run the RS-232 output (USART1) at 921600 baud instead of 115200.
The result is placed in *build\\<variant>[-dronecan]\\*: *target-app-bahrs-v2.elf*, *.hex* and *.map*. Flash the *.hex* file, not the *.elf*.
