# BAHRS software release PF_2_0_beta1

This is the first beta-version of the release PF_2_0. The release has got the latest features and is ideal for early-adopters.

## Content

1. Firmware (.hex file)
2. Default NVM image: use the image from the release PF_1_0
3. NVM map file: use the image from the release PF_1_0
4. Serial protocol converter: see the subfolder *Utilities* from the release PF_1_0

## Release notes

1. Introduced three independent Kalman filters for attitude and vertical motion estimation
2. Implemented fault detection and isolation capability for roll and pitch signals
3. Implemented fault detection and isolation for altitude and vertical speed signals
4. Implemented fault detection and isolation for pressure measurements
5. Performance improvements and bugfixes

## Known issues

1. Attitude monitor has high false alarm rate at gimbal lock (+-90 degrees pitch) and at +-180 degrees roll.
2. The software cannot detect saturation of inertial signals of ICM20789, and thus misleading attitude/vertical channel data output is possible when
   magnitude of angular rate exceeds 500 deg/s, or when acceleration exceeds 5 g.

