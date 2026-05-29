# Introduction

The folder contains source code of the firmware of EULER-NAV Baro-Inertial AHRS.

To develop the firmware we use a combination of CMake and STM32CubeIDE.

# Installation of the build toolchain (Windows)

1. Install CMake 4.0.0, add location of the *cmake.exe* to the system *PATH* variable
2. Install Ninja. It is required to download a the zip archive and unpack it to *C:\Tools\ninja-win\\*.
   After that add the folder *C:\Tools\ninja-win\\* to the system *PATH* variable.
3. Install STM32CubeIDE 1.18.0

# How to build the firmware

1. Open STM32CubeIDE 1.18.0. Create a new workspace or choose the existing.
2. Click *File -> Open projects from file system...*.
3. Choose the directory *<repo root>\02_Projects\Stm32Bahrs\TargetApp_HW_V3*.

![](img/01_open_project.png)

4. Right-click on the imported project, press *CMake Configure*.

![](img/02_cmake_configure.png)

5. Click the "Hammer" icon, select and build the desired project variant.

![](img/03_build_project_variant.png)

6. Re-run *Cmake Configure* whenever you modify any of the *CMakeLists.txt* or *\*.cmake* files.
