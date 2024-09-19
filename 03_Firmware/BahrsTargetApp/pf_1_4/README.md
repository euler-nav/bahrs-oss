# BAHRS software release PF_1_4

This is a pre-built standard BAHRS software that samples are supplied with starting from October 2024.

## Content

1. Firmware (.hex file)
2. Default NVM image: use the image from the release PF_1_0
3. NVM map file: use the image from the release PF_1_0
4. Serial protocol converter: see the subfolder *Utilities* from the release PF_1_0

## Release notes

1. Introduced a workaround for I2C HAL library vulnerability that was causing the device to stop sending data when ICM20789 chip hangs.
2. Raised interrupt priority of TIM6 to ensure that HAL function timeouts work properly.
3. CAN stack performance improvements

## Known issues

None
