# Introduction

The hardware test app firmware (*HwTestApp_V_2_0.hex*) performs two tasks:

- executes a simple hardware test
- streams raw magnetometer measurements for calibration purpose

**IMPORTANT**: the app is programmed to the application memory of BAHRS device (address 0x8008000). The app will not start automatically after power-on, if a bootloader is not programmed to the address zero.

## Communication protocol

The app communicates via RS232. Baud rate 115200, no parity bits, 1 stop bit. The app outputs plain text.

## Simple hardware test

The test consists of the following steps and is executed immediately after power-on:

1. Attempt to initialize sensors
2. Attempt to write to and read from the NVM
3. Send dummy frames on CAN
4. Detect sync pulse

## Magnetometer data streaming

To trigger magnetometer data streaming send the following ASCII string "START MAGDATA\r\n".

To return to idle mode send the command "STOP\r\n".

The app acknowledges successful mode transition by printing confirmations

- MODE IDLE
- MODE MAGDATA

Sample output after power-on:

- Sensor BMP384 initialization OK
- Sensor MMC5983 initialization OK
- Sensor SCHA63T initialization OK
- Sensor BMM150-1 initialization OK
- Sensor BMM150-2 initialization OK
- Sensor ICM20789-1 initialization OK
- Sensor ICM20789-2 initialization OK
- NVM test OK
- CAN transmission SUCCESS. Attempts left: 4.
- CAN transmission SUCCESS. Attempts left: 3.
- CAN transmission SUCCESS. Attempts left: 2.
- CAN transmission SUCCESS. Attempts left: 1.
- CAN transmission SUCCESS. Attempts left: 0.

After triggering magnetometer data streaming output will look like this(format: MAG < MODEL > < INDEX > < FieldX > < FieldY > < FieldZ >):

- MODE MAGDATA
- MAG BMM150 1 0.110000 0.570000 0.830000
- MAG BMM150 2 0.450000 0.070000 -3.340000
- MAG MMC5983 1 0.492249 0.017273 0.826843
- MAG BMM150 1 0.130000 0.580000 0.840000
- MAG BMM150 2 0.450000 0.070000 -3.360000

