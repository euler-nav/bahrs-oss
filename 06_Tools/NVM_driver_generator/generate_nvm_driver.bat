@echo off
REM @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
REM Batch script to run NVM driver generator

python nvm_driver_generator.py
if errorlevel 1 goto end

copy /Y CNvmDriver.h ..\..\02_Projects\Stm32Bahrs\shared\bsw\swc-nvm-driver\inc\CNvmDriver.h

:end
pause
