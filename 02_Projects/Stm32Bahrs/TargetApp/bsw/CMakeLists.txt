
add_subdirectory(bmp384-driver)
add_subdirectory(scha63t-driver)
add_subdirectory(mmc5983-driver)
add_subdirectory(bmm150-driver)
add_subdirectory(icm20789-driver)

# The diagnostic-mode submodule has no CMakeLists.txt in this layout, so the target is defined here.
add_library(swc-diagnostic-mode STATIC
    "diagnostic-mode/CDiagnosticMode.cpp"
    "diagnostic-mode/CDiagnosticModeWrapper.cpp")

target_include_directories(swc-diagnostic-mode PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/diagnostic-mode")

target_link_libraries(swc-diagnostic-mode PUBLIC swc-nvm-driver)

add_subdirectory("${CMAKE_SOURCE_DIR}/../shared/bsw" "${CMAKE_CURRENT_BINARY_DIR}/shared/bsw/")
