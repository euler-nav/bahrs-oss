# BAHRS software release PF_1_1

This is a pre-built standard BAHRS software that samples are supplied with starting from January 2024. For pre-built open source version please refer to the 03_Firmware folder.

## Content

1. Firmware (.hex file)
2. Default NVM image: use the image from the release PF_1_0
3. NVM map file: use the image from the release PF_1_0
4. Serial protocol converter: see the subfolder *Utilities* from the release PF_1_0

## Release notes

1. Implemented safe IMU signal chain. The firmware outputs IMU measurements if and only if signals of all redundant IMUs agree, OR if a faulty sensor was successfully isolated.
2. Improved configuration of the primary magnetometer chip. The change results into mitigation of turn-on drift of magnetic heading.
3. Minor output protocol update: added the "Software version" message 0x0F that is sent once after turn-on.

## Known issues

Extreme variation of temperature during operation may result into frequent switches to signals of back-up inertial sensors causing larger noise.

An example of extreme temperature variation: the sample cooled down to -20 degrees Celsius and powered on at +20.
