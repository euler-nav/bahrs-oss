set(LIB_NAME swc-diagnostic-mode)

add_library(${LIB_NAME} STATIC "src/CDiagnosticMode.cpp" "src/CDiagnosticModeWrapper.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")
    
target_link_libraries(${LIB_NAME} PUBLIC swc-nvm-driver)
