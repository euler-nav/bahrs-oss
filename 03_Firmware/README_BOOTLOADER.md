# BAHRS bootloader
## Introduction

The bootloader firmware (*BahrsBootloader_V_1_0.hex*) enables updates of BAHRS firmware. The device's flash memory can be programmed with the help of STM32CubeProgrammer. After power-on the bootloader waits for a connection request from a host. After a timeout of 0.5 second elapses, the bootloader will start a firmware located in the memory address 0x8008000.
