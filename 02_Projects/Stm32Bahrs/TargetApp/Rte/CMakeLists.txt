
# The target name is kept as "bahrs-v3-rte" because the shared SWC/BSW libraries and the BSW driver
# submodules link this exact name. It is the RTE of hardware V2 in this project.
set(LIB_NAME bahrs-v3-rte)

set(RTE_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rte_generated")

add_library(${LIB_NAME} STATIC
    "${RTE_GENERATED_DIR}/Src/CRte.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Src/RteWrapper.cpp"
    "${CMAKE_SOURCE_DIR}/../shared/lib/swc-ports/CSoftwareComponentPort.cpp")

# Rte/Inc must precede rte_generated/Inc: both contain an RteTypesGenerated.h
target_include_directories(${LIB_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/Inc"
    "${RTE_GENERATED_DIR}/Inc"
    "${CMAKE_SOURCE_DIR}/../shared/lib/swc-ports"
    "${CMAKE_SOURCE_DIR}/../shared/lib/common")

target_link_libraries(${LIB_NAME} PUBLIC free-rtos-cmsis-os
    swc-can-common
    swc-rs232-output-handler
    swc-bmp384-driver
    swc-bmm150-driver
    swc-scha63t-driver
    swc-icm20789-driver
    swc-mmc5983-driver
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
    os-call-wrappers
    ams-assertion-handler)

# swc-can-output-handler/swc-drone-can each link bahrs-v3-rte back (CRte access), so this is a
# deliberate cyclic PUBLIC dependency; CMake resolves it fine for static libs and include dirs.
if(USE_DRONECAN)
    target_link_libraries(${LIB_NAME} PUBLIC swc-drone-can)
else()
    target_link_libraries(${LIB_NAME} PUBLIC swc-can-output-handler)
endif()
