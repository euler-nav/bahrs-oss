# BAHRS software release PF_1_5

This is a pre-built standard EULER-NAV Baro-Inertial AHRS software that the devices are shipped with starting from January 2025.

## Content

1. Firmware (.hex file)
2. Default NVM image: use the image from the release PF_1_0
3. NVM map file: use the image from the release PF_1_0
4. Serial protocol converter: see the subfolder *Utilities* from the release PF_1_0

## Release notes

1. Sensor fusion library upgrade
2. Re-initialize ICM20789 chips if the I2C communication stops working.
3. Fix encoding of IMU data outputs on RS232 interface.

## Known issues

None
