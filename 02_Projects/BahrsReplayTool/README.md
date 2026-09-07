# EULER-NAV Baro-Inertial AHRS Replay Tool Demo

## Introduction and Purpose

This replay tool demo is an Octave harness built around a pre-compiled executable that replays BAHRS software components and their interactions
using extended debug output that includes:

- port write event messages
- runnable call event messages

Port write event messages contain timestamp and payload of software component (SWC) port write events. Runnable call events mark trigger times
of functions of interest. The pre-compiled executable reads these messages from a binary flight log and resimulates behavior of BAHRS software
components.

## Requirements

- Windows
- Octave 9.3.0 or newer

The tool is also expected to run in Matlab.

## How to run

Open the file *script_runBahrsFilterReplay.m* in Octave GUI, then click the "Play" button. After a while, the script will plot selected results
of the software re-simulation. Note that scaling and zooming the plots in Octave will likely be slow.

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

## Have questions of need support?

Write an email to *info@euler-nav.com*.
