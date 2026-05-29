cmake_minimum_required(VERSION 4.0)

set(PROJECT_NAME_STRING BAHRS)

project(${PROJECT_NAME_STRING} C CXX ASM)

option(ENABLE_PRINTF "Enable newlib-nano malloc wrappers for printf debugging" OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "debug" CACHE STRING "Choose the type of build." FORCE)
endif()

if(ENABLE_PRINTF)
    add_compile_definitions(ENABLE_PRINTF)   # applies to all targets in this directory
endif()

# Import Eigen library
add_library(eigen INTERFACE)
target_include_directories(eigen INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/../../../99_3rd_party/eigen-3.4.0")

# Import sensor fusion library
add_library(bahrs-sensor-fusion-library STATIC IMPORTED)

set_target_properties(bahrs-sensor-fusion-library PROPERTIES
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/../../../01_Library/Binaries/libBahrsSensorFusionLibrarySTM32.a"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/../../../01_Library/Headers/BahrsSensorFusion"
)

target_link_libraries(bahrs-sensor-fusion-library INTERFACE eigen)

# Collect all parts of the software
add_subdirectory(bahrs-v3-cube/Drivers/STM32F4xx_HAL_Driver)
add_subdirectory(bahrs-v3-cube/Drivers/CMSIS)
add_subdirectory(bahrs-v3-cube/Middlewares/Third_Party/FreeRTOS)
add_subdirectory(bahrs-v3-rte)
add_subdirectory(bsw)
add_subdirectory(asw)
add_subdirectory(stm32-lib-common)
add_subdirectory(libs/ams-assertion-handler)
add_subdirectory(libs/serial-protocol)
add_subdirectory(libs/os-call-wrappers)

set(EXECUTABLE_NAME target-app-bahrs-v3)

file(GLOB BAHRS_V3_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/bahrs-v3-cube/Core/Src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/bahrs-v3-cube/Core/Startup/*.s"
    "${CMAKE_CURRENT_SOURCE_DIR}/bahrs-v3-cube/Core/Src/*.cpp"
    "${CMAKE_SOURCE_DIR}/../shared/tasks/tasks_asw.cpp"
    "${CMAKE_SOURCE_DIR}/../shared/lib/ams-dynamic-memory-functions/ams_std_memory_functions.cpp")

add_executable(${EXECUTABLE_NAME}.elf ${BAHRS_V3_SOURCES} )

target_include_directories(${EXECUTABLE_NAME}.elf PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/bahrs-v3-cube/Core/Inc"
)

target_link_libraries(${EXECUTABLE_NAME}.elf bahrs-v3-rte ams-assertion-handler bahrs-sensor-fusion-library)

target_link_options(${EXECUTABLE_NAME}.elf
    PRIVATE
    -T${CMAKE_SOURCE_DIR}/bahrs-v3-cube/STM32F446ZEJX_FLASH.ld
    -Wl,-Map=${CMAKE_BINARY_DIR}/${EXECUTABLE_NAME}.map
)

if(ENABLE_PRINTF)
    target_link_options(${EXECUTABLE_NAME}.elf PRIVATE
        -Wl,--wrap=_malloc_r
        -Wl,--wrap=_free_r
    )
endif()

# Generate HEX and BIN files
add_custom_command(TARGET ${EXECUTABLE_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${EXECUTABLE_NAME}.elf> ${EXECUTABLE_NAME}.hex
    COMMENT "Generating HEX"
)
