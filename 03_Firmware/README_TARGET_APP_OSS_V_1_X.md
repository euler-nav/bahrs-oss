# Introduction

*BahrsTargetApp_OSS_V_1_0.hex* and *BahrsTargetApp_OSS_V_1_1.hex* are precompiled open-source firmware for BAHRS rev. 2. They predate the firmware in this repository; for new work, use the latest release in [BahrsTargetApp](BahrsTargetApp/README.md).

**IMPORTANT**: the app is programmed to the application memory of BAHRS device (address 0x8008000). The app will not start automatically after power-on, if a bootloader is not programmed to the address 0x8000000.

## Communication protocol

The app communicates via RS232. Baud rate 115200, no parity bits, 1 stop bit.

For binary protocol documentation refer to [the official documentation page](https://euler-nav.com/bahrsdoc).

