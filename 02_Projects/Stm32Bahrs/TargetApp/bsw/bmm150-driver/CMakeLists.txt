set(LIB_NAME swc-bmm150-driver)

add_library(${LIB_NAME} STATIC "src/CBmm150Driver.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")

target_link_libraries(${LIB_NAME} PUBLIC bahrs-v3-rte hal-stm32f4 stm32-lib-common)
