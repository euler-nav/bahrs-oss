# BAHRS bootloader
## Introduction

The bootloader firmware (*BahrsBootloader_V_1_0.hex*) is for BAHRS rev. 2 and enables updates of BAHRS firmware. Devices ship with the bootloader, so you only need this file if it was erased. In that case, program it with an ST-LINK through the debug connector (SWD, see the [knowledge base](../00_Documentation/KNOWLEDGE_BASE.md#debug-connector-schematics)).

The device's flash memory can be programmed with the help of STM32CubeProgrammer. After power-on the bootloader waits for a connection request from a host. After a timeout of 0.5 second elapses, the bootloader will start a firmware located in the memory address 0x8008000.
