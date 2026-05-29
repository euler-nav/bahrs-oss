set(LIB_NAME swc-spi-handler)

add_library(${LIB_NAME} STATIC "src/CSpiHandler.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")

target_link_libraries(${LIB_NAME} PUBLIC bahrs-v3-rte ams-assertion-handler os-call-wrappers)
