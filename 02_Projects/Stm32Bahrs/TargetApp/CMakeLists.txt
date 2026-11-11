cmake_minimum_required(VERSION 4.0)

set(PROJECT_NAME_STRING BAHRS)

project(${PROJECT_NAME_STRING} C CXX ASM)

option(ENABLE_PRINTF "Enable newlib-nano malloc wrappers for printf debugging" OFF)
option(USE_DRONECAN "Build the DroneCAN CAN stack instead of CANaerospace" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "debug" CACHE STRING "Choose the type of build." FORCE)
endif()

set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "debug" "release" "debug-with-debug-output")

if(CMAKE_BUILD_TYPE STREQUAL "debug-with-debug-output")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_DEBUG_WITH_DEBUG_OUTPUT}" CACHE STRING "C flags" FORCE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG_WITH_DEBUG_OUTPUT}" CACHE STRING "C++ flags" FORCE)
    set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS_DEBUG_WITH_DEBUG_OUTPUT}" CACHE STRING "ASM flags" FORCE)
endif()

# The former CDT "Debug" configuration of V2 always enabled ENABLE_PRINTF
if(CMAKE_BUILD_TYPE STREQUAL "debug")
    set(ENABLE_PRINTF ON CACHE BOOL "Enable newlib-nano malloc wrappers for printf debugging" FORCE)
endif()

if(ENABLE_PRINTF)
    add_compile_definitions(ENABLE_PRINTF)   # applies to all targets in this directory
endif()

if(USE_DRONECAN)
    add_compile_definitions(USE_DRONECAN)    # selects the DroneCAN stack in CRte and the task routines
else()
    add_compile_definitions(CAN_FREERTOS=1 CAN_RX_QUEUE_LEN=128)    # CANaerospace STM32 driver options, unused by DroneCAN
    # Not defined for DroneCAN: _DEFAULT_SOURCE exposes usleep() from <unistd.h>, which clashes with usleep_shim.h
    add_compile_definitions(MEASURE_RESOURCES_USAGE _DEFAULT_SOURCE)
endif()

# Import Eigen library
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../../99_3rd_party/eigen-3.4.0")

# Import sensor fusion library (the "d" variant is used by the debug build types, as in the former CDT project)
if(CMAKE_BUILD_TYPE STREQUAL "release")
    set(SENSOR_FUSION_LIB_FILE "libBahrsSensorFusionLibrarySTM32.a")
else()
    set(SENSOR_FUSION_LIB_FILE "libBahrsSensorFusionLibrarySTM32d.a")
endif()

add_library(bahrs-sensor-fusion-library STATIC IMPORTED)

set_target_properties(bahrs-sensor-fusion-library PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../shared/lib/bahrs-sensor-fusion/Binaries/${SENSOR_FUSION_LIB_FILE}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/../shared/lib/bahrs-sensor-fusion/Headers/BahrsSensorFusion"
)

target_link_libraries(bahrs-sensor-fusion-library INTERFACE eigen)

# Collect all parts of the software
set(BAHRS_CUBE_DIR "${CMAKE_SOURCE_DIR}/bahrs-v2-cube")   # used by shared CMake files
add_subdirectory(bahrs-v2-cube/Drivers/STM32F4xx_HAL_Driver)
add_subdirectory(bahrs-v2-cube/Drivers/CMSIS)
add_subdirectory(bahrs-v2-cube/Middlewares/Third_Party/FreeRTOS)
add_subdirectory(Rte)
add_subdirectory(bsw)
add_subdirectory("${CMAKE_SOURCE_DIR}/../shared/asw" "${CMAKE_CURRENT_BINARY_DIR}/shared/asw/")
add_subdirectory(lib)

set(EXECUTABLE_NAME target-app-bahrs-v2)

file(GLOB BAHRS_V2_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/bahrs-v2-cube/Core/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/bahrs-v2-cube/Core/Startup/*.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/Rte/Src/TaskRoutines.cpp"
    "${CMAKE_SOURCE_DIR}/../shared/tasks/tasks_asw.cpp"
    "${CMAKE_SOURCE_DIR}/../shared/lib/ams-dynamic-memory-functions/ams_std_memory_functions.cpp")

add_executable(${EXECUTABLE_NAME}.elf ${BAHRS_V2_SOURCES})

target_include_directories(${EXECUTABLE_NAME}.elf PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/bahrs-v2-cube/Core/Inc"
)

target_link_libraries(${EXECUTABLE_NAME}.elf bahrs-v3-rte ams-assertion-handler bahrs-sensor-fusion-library swc-diagnostic-mode)

if(USE_DRONECAN)
    target_link_libraries(${EXECUTABLE_NAME}.elf swc-drone-can)
else()
    target_link_libraries(${EXECUTABLE_NAME}.elf swc-can-output-handler)
endif()

target_link_options(${EXECUTABLE_NAME}.elf
    PRIVATE
    -T${CMAKE_SOURCE_DIR}/bahrs-v2-cube/STM32F446RETX_FLASH.ld
    -Wl,-Map=${CMAKE_BINARY_DIR}/${EXECUTABLE_NAME}.map
    -Wl,--gc-sections
)

if(ENABLE_PRINTF)
    target_link_options(${EXECUTABLE_NAME}.elf PRIVATE
        -Wl,--wrap=_malloc_r
        -Wl,--wrap=_free_r
    )
endif()

# Generate HEX file
add_custom_command(TARGET ${EXECUTABLE_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE_NAME}.elf> ${EXECUTABLE_NAME}.hex
    COMMENT "Generating HEX"
)
