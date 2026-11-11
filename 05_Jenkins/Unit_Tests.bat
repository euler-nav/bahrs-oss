@echo off
REM @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

setlocal

set UT_EXECUTABLE_BUILD_PLATFORM_FOLDER="02_Projects\VisualStudioToolsBahrs\BahrsUT\x64"
set UT_EXECUTABLE_NAME=BahrsUT.exe
set BUILD_VARIANT=%1

%UT_EXECUTABLE_BUILD_PLATFORM_FOLDER%\%BUILD_VARIANT%\%UT_EXECUTABLE_NAME% --gtest_shuffle --gtest_repeat=5 --gtest_random_seed=17

endlocal