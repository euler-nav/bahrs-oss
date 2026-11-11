set(LIB_NAME swc-can-common)

add_library(${LIB_NAME} INTERFACE)

target_include_directories(${LIB_NAME} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/inc")
