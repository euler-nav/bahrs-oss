# Knowledge base

Here you find some useful manuals.

## How to reprogramm BAHRS via RS232

To reprogramm the device use STM32CubeProgrammer. The procedure described below was confirmed to work with STM32CubeProgrammer v2.13.0.

Reprogramming procedure:

1. Start STM32CubeProgrammer, configure UART to operate at 115200 baud rate, no parity bits, 1 stop bit.

![](img/Start_STM32CubeProgrammer.png)

2. Connect BAHRS to the host's COM port

3. Power-on the BAHRS

4. Click the "Connect" button. Shall be done within 0.5 seconds after power-on.

![](img/Start_STM32CubeProgrammer_Connect.png)

5. Confirm that connection was successful.

![](img/Start_STM32CubeProgrammer_Connection_Successful.png)

6. Go to the reprogramming tab.

![](img/Start_STM32CubeProgrammer_Reprogramming.png)

7. Select a HEX file to upload to the device. **IMPORTANT**: do **NOT** select **ELF** files. It was observed that uploading ELF files leads to erasing all the sectors of the microcontroller's FLASH. Consequently, a bootloader will be erased and the application will not start.

8. Click "Start programming", wait for completion, and confirm that the sequence ended with no errors.

![](img/Start_STM32CubeProgrammer_Reprogramming_in_Progress.png)

![](img/Start_STM32CubeProgrammer_Reprogramming_Successful.png)

9. Disconnect.

![](img/Start_STM32CubeProgrammer_Disconnect.png)

10. Reset the power of the BAHRS

11. Your new firmware will now run.

## Debug connector schematics

Debug interface: SWD.

Connector part number: BC075-08-A-L-A.

Pin assignment:

1. SWDIO
2. GND
3. SWCLK
4. +3.3V
5. SWO
6. UART TX
7. UART RX
8. NRST

![](img/Debug_connector.png)

