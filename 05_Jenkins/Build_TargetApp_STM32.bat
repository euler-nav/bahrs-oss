@echo off

setlocal

set STM32_CUBE_HEADLESS_BAT_PATH="C:\ST\STM32CubeIDE_1.9.0\STM32CubeIDE\headless-build.bat"
set PROJECT_PATH_REL="02_Projects\Stm32Bahrs\TargetApp"
set BUILD_VARIANT=%1

%STM32_CUBE_HEADLESS_BAT_PATH% -import %PROJECT_PATH_REL% -cleanBuild BahrsTargetApp/%BUILD_VARIANT%

endlocal