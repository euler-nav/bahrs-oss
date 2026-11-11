set(LIB_NAME swc-drone-can)

# Minimal libcanard static library -- compiled as C code separately.
# We do not use libcanard's own CMakeLists.txt to avoid its project() call and
# host-build flags which are incompatible with the ARM cross-compilation setup.
# Only canard.c (core) and the STM32 bxCAN driver are compiled; examples, tests
# and all other platform drivers are excluded.
add_library(lib-canard STATIC
    "${CMAKE_SOURCE_DIR}/../../../99_3rd_party/libcanard/canard.c"
    "${CMAKE_SOURCE_DIR}/../../../99_3rd_party/libcanard/drivers/stm32/canard_stm32.c")

target_include_directories(lib-canard PUBLIC
    "${CMAKE_SOURCE_DIR}/../../../99_3rd_party/libcanard")

target_include_directories(lib-canard PRIVATE
    "${CMAKE_SOURCE_DIR}/../../../99_3rd_party/libcanard/drivers/stm32"
    "${BAHRS_CUBE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc"
    "${BAHRS_CUBE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
    "${BAHRS_CUBE_DIR}/Drivers/CMSIS/Include"
    "${BAHRS_CUBE_DIR}/Core/Inc")

# Force-include the usleep shim so canard_stm32.c can call HAL_Delay()
# instead of the unavailable POSIX usleep().  The 3rd-party source is not
# modified.
target_compile_options(lib-canard PRIVATE
    "-include${CMAKE_CURRENT_SOURCE_DIR}/usleep_shim.h")

# Minimal STM32 bxCAN driver for DroneCAN (clean-room; no third-party code).
# Brings up the CAN peripheral with an accept-all filter and does polled TX/RX.
# The CAN instance is selected by CAN_INTERFACE_NUMBER; that and USE_HAL_DRIVER
# come from the toolchain's global definitions.
add_library(lib-drone-can-driver STATIC
    "${CMAKE_CURRENT_SOURCE_DIR}/can-driver/src/bxcan.c")

target_include_directories(lib-drone-can-driver PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/can-driver/inc")

# free-rtos-cmsis-os is linked only for its transitive HAL/CMSIS include paths.
target_link_libraries(lib-drone-can-driver PUBLIC free-rtos-cmsis-os)

# DroneCAN BSW component
add_library(${LIB_NAME} STATIC
    "src/CDroneCanHandler.cpp"
    "src/CDroneCanOutputHandler.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")

target_link_libraries(${LIB_NAME} PUBLIC
    bahrs-v3-rte
    lib-drone-can-driver
    lib-canard)
