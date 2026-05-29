set(LIB_NAME ams-assertion-handler)

add_library(${LIB_NAME} STATIC "src/AmsAssert.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")
    
target_link_libraries(${LIB_NAME} PUBLIC hal-stm32f4)
