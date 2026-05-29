# Target platform
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain root (use forward slashes in CMake, even on Windows)
set(TOOLCHAIN_DIR "C:/ST/STM32CubeIDE_1.18.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin")
set(TOOLCHAIN_PREFIX arm-none-eabi)

# Full paths to the toolchain tools
set(CMAKE_C_COMPILER   "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}-gcc.exe")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}-g++.exe")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}-gcc.exe")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_DIR}/${TOOLCHAIN_PREFIX}-objcopy.exe")

# Set compiler flags common for all variants
set(COMMON_ASSEMBLER_FLAGS "-mcpu=cortex-m4 -x assembler-with-cpp --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(COMMON_C_FLAGS "-mcpu=cortex-m4 -std=c99 -ffunction-sections -fdata-sections -pedantic -pedantic-errors -Wmissing-include-dirs -fstack-usage --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")
set(COMMON_CXX_FLAGS "-mcpu=cortex-m4 -std=c++17 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -pedantic -pedantic-errors -Wmissing-include-dirs -fstack-usage --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb")

# Set common preprocessor definitions
set(COMMON_COMPILE_DEFINITIONS "-DSTM32F446xx -DUSE_HAL_DRIVER -DUSE_FULL_ASSERT -DBAHRS_HW_V3")

# Set variant-specific compiler options
set(CMAKE_ASM_FLAGS_DEBUG "${COMMON_ASSEMBLER_FLAGS}")
set(CMAKE_C_FLAGS_DEBUG   "${COMMON_C_FLAGS} -O3 -g3 ${COMMON_COMPILE_DEFINITIONS}")
set(CMAKE_CXX_FLAGS_DEBUG "${COMMON_CXX_FLAGS} -O3 -g3 ${COMMON_COMPILE_DEFINITIONS}")

set(CMAKE_ASM_FLAGS_RELEASE "${COMMON_ASSEMBLER_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE   "${COMMON_C_FLAGS} -O3 -Werror ${COMMON_COMPILE_DEFINITIONS}")
set(CMAKE_CXX_FLAGS_RELEASE "${COMMON_CXX_FLAGS} -O3 -Werror ${COMMON_COMPILE_DEFINITIONS}")

# Avoid CMake test program builds (useful for embedded)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
