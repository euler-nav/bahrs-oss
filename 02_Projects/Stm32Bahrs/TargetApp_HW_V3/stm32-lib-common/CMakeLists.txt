set(LIB_NAME stm32-lib-common)

add_library(${LIB_NAME} STATIC
    "${CMAKE_SOURCE_DIR}/../../../01_Library/Stm32/Lib/crc32.cpp"
    "${CMAKE_SOURCE_DIR}/../../../01_Library/Stm32/Lib/GetMicroseconds.c")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_SOURCE_DIR}/../../../01_Library/Stm32/Lib")
    
target_link_libraries(${LIB_NAME} PUBLIC hal-stm32f4)
