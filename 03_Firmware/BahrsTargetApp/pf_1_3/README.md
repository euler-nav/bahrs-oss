# BAHRS software release PF_1_3

This is a pre-built standard BAHRS software that samples are supplied with starting from July 2024.

## Content

1. Firmware (.hex file)
2. Default NVM image: use the image from the release PF_1_0
3. NVM map file: use the image from the release PF_1_0
4. Serial protocol converter: see the subfolder *Utilities* from the release PF_1_0

## Release notes

1. Fixed configuration of interrupt priority for TIM6.
2. Improved IMU monitor performance under high temperature gradients.
3. Fixed and issue in the driver of SCHA63T that was causing the device to stop ptoviding output when vehicle rate exceeded measurement range of SCHA63T.

## Known issues

The SW was observed to stop sending output after 1 hour of operation, when active external nodes were connected to the CAN bus simultaneously with EULER-NAV BAHRS.
The issue does not occur if the device is not connected to a CAN bus, or when external CAN nodes only listen and do not send.
