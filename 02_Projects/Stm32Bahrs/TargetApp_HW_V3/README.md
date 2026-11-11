# Introduction

The folder contains source code of the firmware of EULER-NAV Baro-Inertial AHRS.

To develop the firmware we use a combination of CMake and STM32CubeIDE.

# Installation of the build toolchain

The same toolchain builds BAHRS rev. 2 and rev. 3.

- Install [STM32CubeIDE 1.18.0](https://www.st.com/en/development-tools/stm32cubeide.html) for Windows. Accept the suggested STM32 toolchain version during installation.
- Install [CMake 4.0.0](https://cmake.org/files/v4.0/) and [Ninja](https://ninja-build.org/), and add both to `PATH`.
- The *cmake\arm-gcc-toolchain.cmake* files of both revisions expect the GNU Tools for STM32 13.3.rel1 compiler under the default STM32CubeIDE 1.18.0 installation, in *C:\ST\STM32CubeIDE_1.18.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344\tools\bin*. If your installation or plugin folder differs, adjust `TOOLCHAIN_DIR` in the toolchain file.

# How to build the firmware

## From the command line

From the repository root in a Windows command prompt:

```bat
05_Jenkins\Build_TargetApp_STM32_V3.bat <debug|release|debug-with-debug-output> [dronecan]
```

- The variant argument is required and must be lowercase.
- `debug-with-debug-output` enables the extended debug output used by the replay and calibration tools. Such builds run the RS-232 output at 921600 baud instead of 115200.
- The optional `dronecan` argument builds the DroneCAN stack instead of CANaerospace (use with `release`).
- The script **deletes and recreates** the build folder. The result is placed in *build\\<variant>[-dronecan]\\* of this project: *target-app-bahrs-v3.elf*, *.hex* and *.map*.
- For BAHRS rev. 2 see *..\TargetApp\README.md*.

## With STM32CubeIDE

1. Open STM32CubeIDE 1.18.0. Create a new workspace or choose the existing.
2. Click *File -> Open projects from file system...*.
3. Choose the directory *<repo root>\02_Projects\Stm32Bahrs\TargetApp_HW_V3*.

![](img/01_open_project.png)

4. Right-click on the imported project, press *CMake Configure*.

![](img/02_cmake_configure.png)

5. Click the "Hammer" icon, select and build the desired project variant.

![](img/03_build_project_variant.png)

6. Re-run *CMake Configure* whenever you modify any of the *CMakeLists.txt* or *\*.cmake* files.

To flash the result, use the *.hex* file as described in the [root README](../../../README.md#flash-a-device).
