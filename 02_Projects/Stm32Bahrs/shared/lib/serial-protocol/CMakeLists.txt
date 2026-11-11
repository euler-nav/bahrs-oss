set(LIB_NAME lib-serial-protocol)

add_library(${LIB_NAME} STATIC "src/CSerialMessageBuilder.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")

target_link_libraries(${LIB_NAME} PUBLIC bahrs-v3-rte bahrs-sensor-fusion-library PRIVATE hal-stm32f4 free-rtos-cmsis-os ams-assertion-handler)
