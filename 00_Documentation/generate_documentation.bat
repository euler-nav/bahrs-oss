@echo off
REM @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

set DOXYGEN_EXE=..\99_3rd_party\doxygen\doxygen.exe

REM Save the directory where this script is located
set SCRIPT_DIR=%~dp0
set VARIANT=%1
set OUTPUT_DIR="output_%VARIANT%"
set ERROR_LOG="%OUTPUT_DIR%\doc_errors.txt"

REM Change to the script's directory
pushd "%SCRIPT_DIR%"

REM Remove output directory if it exists
if exist "output" (
    echo Removing existing output directory...
    rmdir /s /q %OUTPUT_DIR%
)

REM Create fresh output directory
echo Creating output directory...
mkdir %OUTPUT_DIR%

%DOXYGEN_EXE% Doxyfile_BAHRS_%VARIANT%

rem Capture doxygen exit code
set DOXYGEN_RETURN_VALUE=%ERRORLEVEL%

if NOT "%DOXYGEN_RETURN_VALUE%"=="0" (
	echo Doxygen returned error level %DOXYGEN_RETURN_VALUE%.
	if exist %ERROR_LOG% (
		echo ----- contents of %ERROR_LOG% -----
		type %ERROR_LOG%
		echo ----- end of %ERROR_LOG% -----
	) else (
		echo No %ERROR_LOG% found.
	)
)

REM Return to the original directory
popd

exit /b %DOXYGEN_RETURN_VALUE%
