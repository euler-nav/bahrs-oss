set(LIB_NAME hal-stm32f4)

file(GLOB LIB_HAL_STM32F4_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Src/*.c")

add_library(${LIB_NAME} STATIC ${LIB_HAL_STM32F4_SOURCES})

target_include_directories(${LIB_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/Inc"
    "${CMAKE_SOURCE_DIR}/bahrs-v3-cube/Core/Inc")
    
target_link_libraries(${LIB_NAME} PUBLIC cmsis-stm32f4)
