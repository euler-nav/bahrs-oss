set(LIB_NAME cmsis-stm32f4)

add_library(${LIB_NAME} INTERFACE)

target_include_directories(${LIB_NAME} INTERFACE
    "Device/ST/STM32F4xx/Include"
    "Include")
