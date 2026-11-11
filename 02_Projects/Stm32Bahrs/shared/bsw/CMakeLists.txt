
add_subdirectory(swc-sync-pulse-handler)
add_subdirectory(swc-rs232-output-handler)
add_subdirectory(swc-nvm-driver)
add_subdirectory(swc-can-common)

if(USE_DRONECAN)
    add_subdirectory(swc-drone-can)
else()
    add_subdirectory(swc-can-aero/CANaerospace)
    add_subdirectory(swc-can-aero)
endif()
