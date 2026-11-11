# EULER-NAV Baro-Inertial AHRS Replay Demo

## Introduction and Purpose

This demo is an Octave harness around the BAHRS replay executable. It replays BAHRS software components and their interactions
using extended debug output that includes:

- port write event messages
- runnable call event messages

Port write event messages contain timestamp and payload of software component (SWC) port write events. Runnable call events mark trigger times
of functions of interest. The executable reads these messages from a binary flight log and resimulates behavior of BAHRS software
components.

The executables are not in this folder. The script uses the ones checked in under *..\VisualStudioToolsBahrs\WorkDir*; see the [root README](../../README.md#build-and-run-the-replay-tools) to rebuild them.

## Requirements

- Windows
- GNU Octave 11.3.0 is the version we use.
- [Microsoft Visual C++ Redistributable for Visual Studio 2015-2022 (x64)](https://aka.ms/vs/17/release/vc_redist.x64.exe) for the MAT converter, unless Visual Studio is installed.

The tool is also expected to run in Matlab.

## How to run

Set the Octave current directory to this folder, open *script_runBahrsFilterReplay.m* in Octave GUI, then click the "Play" button. After a while, the script will plot selected results
of the software re-simulation. Note that scaling and zooming the plots in Octave will likely be slow.

The *data* folder contains two BAHRS rev. 3 recordings. The script uses the first one; to try the other, change `testConfig.binaryFile`. Replay results are written next to them as *resim_\*.bin*.

The magnetometer and pressure calibration used by the replay is hard-coded for these two recordings. To replay your own recordings, calibrate with [mag_calib.m](../../06_Tools/MagCalib/mag_calib.m), put the results into *CMagnetometerCompensator.cpp* and *CPressureCompensator.cpp* and rebuild the replay tool.

## Hints to the re-simulation

Once re-play is completed, all the history of the internal SWC ports will be loaded into Octave's workspace allowing a user to create
custom plots and explore what happens and when. The images below show how to navigate to SWC port content in the Octave's variable editor.

![](img/SWC_port_data_location.png)

![](img/SWC_port_data_example.png)

The list of software component ports mirrors the UML component diagram of the BAHRS application software documented on the
[public high-level software architecture pages](https://euler-nav.github.io/bahrs-arch-docs/page_building_block_view.html).

The image below shows how filtered IMU data ports map to the Octave workspace.

![](img/SW_arch_screenshot.png)

![](img/Where_to_find_ports.png)

## Have questions or need support?

Write an email to *info@euler-nav.com*.
