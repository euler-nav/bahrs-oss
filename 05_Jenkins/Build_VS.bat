@echo off
REM @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

setlocal

set SOLUTION_FILE="02_Projects\VisualStudioToolsBahrs\VisualStudioToolsBahrs.sln"
set BUILD_VARIANT=%1

:: Note that MSBuild.exe shall be in system path
MSBuild.exe %SOLUTION_FILE% -t:rebuild -p:Configuration=%BUILD_VARIANT%

endlocal