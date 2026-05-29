# Introduction

This is the open-source software of the [EULER-NAV BAHRS](https://euler-nav.com/bahrs) Miniature Baro-Inertial Attitude and Heading Reference System.

![](00_Documentation/img/BAHRS_main_image.jpg)

# Licenses

The software includes third-party open-source modules. For copyright notices please refer to NOTICES.txt.

Modules developed by AMS Advanced Air Mobility Sensors UG are located in the subfolders

- *01_Library*
- *02_Projects\Stm32Bahrs\TargetApp\Rte*
- *02_Projects\Stm32Bahrs\TargetApp_HW_V3*
- *02_Projects\BahrsEvaluationTool*
- *06_Tools*

The modules are distributed under the 3-clause BSD license.

# Target project and IDE

## BAHRS HW rev. 2

Location: *02_Projects\Stm32Bahrs\TargetApp*.

IDE: STM32CubeIDE 1.9.0.

To open the project start the IDE, select File -> Import Projects from File System or Archive -> Directory. Navigate to and select the path "< cloned repo path >\02_Projects\Stm32Bahrs\TargetApp", click "Finish".

## BAHRS HW rev. 3

Location: *02_Projects\Stm32Bahrs\TargetApp_HW_V3*.

IDE: STM32CubeIDE 1.18.0 + CMake + Ninja.

To open the project start the IDE, select File -> Import Projects from File System or Archive -> Directory. Navigate to and select the path "< cloned repo path >\02_Projects\Stm32Bahrs\TargetApp_HW_V3", click "Finish".

See the README in the project location for toolchain installation instructions.

# How to plot recorded serial binary logs

Use the Octave script located in *02_Projects\BahrsEvaluationTool*.

# Documentation

See .md files in the subfolder *00_Documentation* and check the official [BAHRS documentation page](https://euler-nav.com/bahrsdoc).