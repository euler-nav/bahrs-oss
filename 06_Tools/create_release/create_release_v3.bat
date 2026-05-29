@echo off
setlocal

REM Set the directory where the script is located
set SCRIPT_DIR=%~dp0
cd /d "%SCRIPT_DIR%\..\.."
set REPO_ROOT=%CD%

REM Define key directories and files
set TARGETAPP_V3_DIR=%REPO_ROOT%\02_Projects\Stm32Bahrs\TargetApp_HW_V3
set RTE_JSON=%REPO_ROOT%\02_Projects\Stm32Bahrs\TargetApp_HW_V3\bahrs-v3-rte\rte.json
set RELEASES_DIR=%SCRIPT_DIR%
set VERSION=%1

REM Check if version is provided
if "%VERSION%"=="" (
    echo Usage: %0 version
    exit /b 1
)

REM Create releases directory if it doesn't exist
if not exist "%RELEASES_DIR%" (
    mkdir "%RELEASES_DIR%"
)

REM Debug: Print paths to verify correctness
echo REPO_ROOT: %REPO_ROOT%
echo RELEASES_DIR: %RELEASES_DIR%
echo TARGETAPP_V3_DIR: %TARGETAPP_V3_DIR%
echo RTE_JSON: %RTE_JSON%

REM Clean the repository
echo Cleaning repository and checking out submodules...
git clean -xdf & git submodule update --init --recursive
if errorlevel 1 (
    echo Failed to clean the repository!
    exit /b 1
)

REM Build the project using build scripts
echo Building projects...

REM Build TargetApp V3 Release
call "%REPO_ROOT%\05_Jenkins\Build_TargetApp_STM32_V3.bat" release
if errorlevel 1 (
    echo TargetApp V3 Release build failed!
    exit /b 1
)

REM Create a new release directory
set RELEASE_DIR=%RELEASES_DIR%\release-%VERSION%
mkdir "%RELEASE_DIR%"
mkdir "%RELEASE_DIR%\Tools"

REM Copy TargetApp V3 artifacts
set V3_RELEASE_HEX=%TARGETAPP_V3_DIR%\build\release\target-app-bahrs-v3.hex

if not exist "%V3_RELEASE_HEX%" (
    echo ERROR: TargetApp V3 Release artifact not found at:
    echo %V3_RELEASE_HEX%
    exit /b 1
)

copy "%V3_RELEASE_HEX%" "%RELEASE_DIR%\bahrs-v3-release-%VERSION%.hex"
if errorlevel 1 (
    echo Failed to copy TargetApp V3 Release artifact!
    exit /b 1
)

REM Copy RTE generator JSON
echo Copying RTE generator JSON...
if not exist "%RTE_JSON%" (
    echo ERROR: RTE generator JSON not found at:
    echo %RTE_JSON%
    exit /b 1
)
copy "%RTE_JSON%" "%RELEASE_DIR%\"
if errorlevel 1 (
    echo Failed to copy RTE generator JSON!
    exit /b 1
)

REM Archive the release directory
echo Creating zip archive...
cd "%RELEASES_DIR%"
powershell Compress-Archive -Path "release-%VERSION%\*" -DestinationPath "bahrs-targetapp-tools-v3-release-%VERSION%.zip" -Force

REM Check if the archive was created successfully
if exist "bahrs-targetapp-tools-v3-release-%VERSION%.zip" (
    echo Release %VERSION% created successfully.
    REM Cleanup temporary files and directories
    rmdir /s /q "%RELEASE_DIR%"
) else (
    echo Failed to create the archive!
    exit /b 1
)

endlocal
exit /b 0
