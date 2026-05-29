set(LIB_NAME os-call-wrappers)

add_library(${LIB_NAME} STATIC "${CMAKE_CURRENT_SOURCE_DIR}/src/CMutex.cpp")

target_include_directories(${LIB_NAME} PUBLIC PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")
    
target_link_libraries(${LIB_NAME} PUBLIC free-rtos-cmsis-os)
