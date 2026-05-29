@echo off
setlocal

REM Set the directory where the script is located
set SCRIPT_DIR=%~dp0
cd /d "%SCRIPT_DIR%\..\.."
set REPO_ROOT=%CD%

REM Define key directories and files
set TARGETAPP_DIR=%REPO_ROOT%\02_Projects\Stm32Bahrs\TargetApp
set RELEASES_DIR=%SCRIPT_DIR%
set BUILD_VARIANT=Debug
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
echo TARGETAPP_DIR: %TARGETAPP_DIR%

REM Clean the repository
echo Cleaning repository...
git clean -xdf
if errorlevel 1 (
    echo Failed to clean the repository!
    exit /b 1
)

REM Build the project using build scripts
echo Building projects...

REM Build TargetApp
call "%REPO_ROOT%\05_Jenkins\Build_TargetApp_STM32.bat" %BUILD_VARIANT%
if errorlevel 1 (
    echo TargetApp %BUILD_VARIANT% build failed!
    exit /b 1
)

REM Create a new release directory
set RELEASE_DIR=%RELEASES_DIR%\release-%VERSION%
mkdir "%RELEASE_DIR%"
mkdir "%RELEASE_DIR%\Tools"

REM Copy TargetApp Release artifact to the root of the release package
echo Copying TargetApp %BUILD_VARIANT% artifact...

REM Copy TargetApp Release artifact
if not exist "%TARGETAPP_DIR%\%BUILD_VARIANT%\BahrsTargetApp.hex" (
    echo ERROR: TargetApp Release artifact not found at:
    echo %TARGETAPP_DIR%\Release\BahrsTargetApp.hex
    exit /b 1
)
copy "%TARGETAPP_DIR%\%BUILD_VARIANT%\BahrsTargetApp.hex" "%RELEASE_DIR%\BahrsTargetApp_%VERSION%.hex" 
if errorlevel 1 (
    echo Failed to copy TargetApp Release artifact!
    exit /b 1
)

REM Archive the release directory
echo Creating zip archive...
cd "%RELEASES_DIR%"
powershell Compress-Archive -Path "release-%VERSION%\*" -DestinationPath "bahrs-targetapp-tools-release-%VERSION%.zip"

REM Check if the archive was created successfully
if exist "bahrs-targetapp-tools-release-%VERSION%.zip" (
    echo Release %VERSION% created successfully.
    REM Cleanup temporary files and directories
    rmdir /s /q "%RELEASE_DIR%"
) else (
    echo Failed to create the archive!
    exit /b 1
)

endlocal
exit /b 0
