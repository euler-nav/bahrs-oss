set(LIB_NAME bahrs-v3-rte)

add_library(${LIB_NAME} STATIC "Src/CRte.cpp" "${CMAKE_SOURCE_DIR}/../../../01_Library/Stm32/Components/CSoftwareComponentPort.cpp")

target_include_directories(${LIB_NAME} PUBLIC 
    "${CMAKE_CURRENT_SOURCE_DIR}/Inc"
    "${CMAKE_SOURCE_DIR}/../../../01_Library/Stm32/Components"
    "${CMAKE_SOURCE_DIR}/../../../01_Library/Common")
    
target_link_libraries(${LIB_NAME} PUBLIC free-rtos-cmsis-os
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
    swc-attitude-monitor
    swc-bahrs-filter
    swc-barometer-monitor
    swc-imu-monitor
    swc-pressure-compensator
    swc-output-transformer
    swc-vertical-channel-monitor
    swc-sync-pulse-handler
    stm32-lib-common
    ams-assertion-handler)
