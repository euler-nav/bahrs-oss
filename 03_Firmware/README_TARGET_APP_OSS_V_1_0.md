# Introduction

*BahrsTargetApp_OSS_V_1_0.hex* is a precompiled open-source firmware.

**IMPORTANT**: the app is programmed to the application memory of BAHRS device (address 0x8008000). The app will not start automatically after power-on, if a bootloader is not programmed to the address zero.

## Communication protocol

The app communicates via RS232. Baud rate 115200, no parity bits, 1 stop bit.

For binary protocol documentation refer to [the official documentation page](https://euler-nav.com/bahrsdoc).

