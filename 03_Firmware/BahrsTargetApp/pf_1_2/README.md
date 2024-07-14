# BAHRS software release PF_1_2

This is a pre-built standard BAHRS software that samples are supplied with starting from May 2024. For pre-built open source version please refer to the 03_Firmware folder.

## Content

1. Firmware (.hex file)
2. Default NVM image: use the image from the release PF_1_0
3. NVM map file: use the image from the release PF_1_0
4. Serial protocol converter: see the subfolder *Utilities* from the release PF_1_0

## Release notes

1. BMP384 pressure sensor driver improvements.
2. Implemented non-blocking polling of ICM-20789 pressure sensor.
3. Fixed a bug that caused software to hang in situations when CAN bus became operational before the system finished initialization.

## Known issues

Extreme variation of temperature during operation may result into frequent switches to signals of back-up inertial sensors causing larger noise.

An example of extreme temperature variation: the sample cooled down to -20 degrees Celsius and powered on at +20.
