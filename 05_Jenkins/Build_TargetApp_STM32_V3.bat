@echo off
setlocal

REM === CONFIGURATION ===
set SCRIPT_DIR=%~dp0
set PROJECT_DIR=%SCRIPT_DIR%\..\02_Projects\Stm32Bahrs\TargetApp_HW_V3
set BUILD_VARIANT=%1
set BUILD_DIR="%PROJECT_DIR%\build\%BUILD_VARIANT%"
set TOOLCHAIN_FILE=%PROJECT_DIR%\cmake\arm-gcc-toolchain.cmake

REM === CLEAN BUILD ===
echo [INFO] Removing previous build: %BUILD_DIR%
rmdir /S /Q %BUILD_DIR%

REM === CREATE NEW BUILD DIR ===
echo [INFO] Creating build directory
mkdir %BUILD_DIR%

REM === CONFIGURE WITH CMAKE ===
echo [INFO] Configuring project...
cmake -S %PROJECT_DIR% -B %BUILD_DIR% -G Ninja -DCMAKE_BUILD_TYPE=%BUILD_VARIANT% -DCMAKE_TOOLCHAIN_FILE=%TOOLCHAIN_FILE% -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

IF ERRORLEVEL 1 (
    echo [ERROR] CMake configuration failed.
    exit /b 1
)

REM === BUILD ===
echo [INFO] Building project...
cmake --build %BUILD_DIR%

IF ERRORLEVEL 1 (
    echo [ERROR] Build failed.
    exit /b 1
)

echo [INFO] Build completed successfully.

endlocal
