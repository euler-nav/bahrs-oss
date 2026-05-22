# BAHRS software release PF_1_8 for hardware rev. 3

This is a pre-built standard EULER-NAV Baro-Inertial AHRS software that the devices are shipped with starting from June 2026.

## Content

1. Firmware (.hex file)
2. Default NVM image: nvm_image_pf_1_8_default.bin
3. NVM map file: nvm_map.json
4. Serial protocol converter: see the folder *..\Tools\*

## Release notes

1. Serial protocol update: signal validity flag replaced with integrity information bitfields
2. Implemented offset compensation between redundant barometers
3. Pressure offset parameters added to NVM
4. Sensor fusion library upgrade: improved AHRS and fault-detection performance


## Known issues

None
