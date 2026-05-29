set(LIB_NAME swc-icp20100-driver)

add_library(${LIB_NAME} STATIC "src/CIcp20100Driver.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")
    
target_link_libraries(${LIB_NAME} PUBLIC bahrs-v3-rte)
