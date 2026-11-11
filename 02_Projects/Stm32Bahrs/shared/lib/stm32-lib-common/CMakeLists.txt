set(LIB_NAME stm32-lib-common)

add_library(${LIB_NAME} STATIC crc32.cpp GetMicroseconds.c)

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(${LIB_NAME} PUBLIC hal-stm32f4)
