set(LIB_NAME swc-lis3mdl-driver)

add_library(${LIB_NAME} STATIC "src/CLis3mdlDriver.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")
    
target_link_libraries(${LIB_NAME} PUBLIC bahrs-v3-rte swc-i2c-handler stm32-lib-common)
