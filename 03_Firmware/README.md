# Prebuilt firmware

All application images are flashed to address 0x8008000 through the bootloader. See [Flash a device](../README.md#flash-a-device) in the root README for the procedure.

| Image | Hardware | Description |
|---|---|---|
| [BahrsTargetApp/pf_2_0/v3](BahrsTargetApp/pf_2_0/v3/README.md) | rev. 3 | Latest standard firmware, with default NVM image and NVM map |
| [BahrsTargetApp/pf_2_0/v2](BahrsTargetApp/pf_2_0/v2/README.md) | rev. 2 | Latest standard firmware, with default NVM image and NVM map |
| [BahrsTargetApp](BahrsTargetApp/README.md) | rev. 2, rev. 3 | All standard firmware releases and their status |
| *BahrsTargetApp_OSS_V_1_x.hex* | rev. 2 | Early open-source firmware, see [README_TARGET_APP_OSS_V_1_X.md](README_TARGET_APP_OSS_V_1_X.md) |
| *HwTestApp_V_2_0.hex* | rev. 2 | Hardware self-test and raw magnetometer streaming, see [README_HW_TEST_APP.md](README_HW_TEST_APP.md) |
| *BahrsBootloader_V_1_0.hex* | rev. 2 | Bootloader at address 0x8000000, see [README_BOOTLOADER.md](README_BOOTLOADER.md) |

If you are unsure, use the latest standard firmware for your hardware revision.
