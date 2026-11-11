set(LIB_NAME bahrs-v3-rte)

set(RTE_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rte_generated")

add_library(${LIB_NAME} STATIC "${RTE_GENERATED_DIR}/Src/CRte.cpp" "${CMAKE_SOURCE_DIR}/../shared/lib/swc-ports/CSoftwareComponentPort.cpp")

target_include_directories(${LIB_NAME} PUBLIC 
    "${RTE_GENERATED_DIR}/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Inc"
    "${CMAKE_SOURCE_DIR}/../shared/lib/swc-ports"
    "${CMAKE_SOURCE_DIR}/../shared/lib/common")

target_link_libraries(${LIB_NAME} PUBLIC free-rtos-cmsis-os
    swc-can-common
    swc-rs232-output-handler
    swc-bmi270-driver
    swc-scha63t-driver
    swc-i2c-handler
    swc-spi-handler
    swc-asm330lhh-driver
    swc-bmp384-driver
    swc-lps22hh-driver
    swc-icp20100-driver
    swc-mmc5983-driver
    swc-lis3mdl-driver
    swc-bmm350-driver
    swc-nvm-driver
    swc-attitude-monitor
    swc-bahrs-filter
    swc-barometer-monitor
    swc-imu-monitor
    swc-magnetic-heading-filter
    swc-magnetic-heading-monitor
    swc-magnetometer-compensator
    swc-imu-pre-filter
    swc-pressure-compensator
    swc-output-transformer
    swc-vertical-channel-monitor
    swc-sync-pulse-handler
    stm32-lib-common
    ams-assertion-handler)

# swc-can-output-handler/swc-drone-can each link bahrs-v3-rte back (CRte access), so this is a
# deliberate cyclic PUBLIC dependency; CMake resolves it fine for static libs and include dirs.
if(USE_DRONECAN)
    target_link_libraries(${LIB_NAME} PUBLIC swc-drone-can)
else()
    target_link_libraries(${LIB_NAME} PUBLIC swc-can-output-handler)
endif()
