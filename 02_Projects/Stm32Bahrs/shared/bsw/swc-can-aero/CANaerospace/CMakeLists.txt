set(LIB_NAME lib-can-aerospace)

add_library(${LIB_NAME} STATIC "drivers/stm32/src/can_driver.c"
    "drivers/stm32/src/can_filter.c"
	"drivers/stm32/src/can_timer.c"
	"canaerospace/src/core.c"
	"canaerospace/src/list.c"
	"canaerospace/src/marshal.c"
	"canaerospace/src/service.c"
	"canaerospace/src/util.c"
	"canaerospace/src/services/std_identification.c")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/canaerospace/include" "${CMAKE_CURRENT_SOURCE_DIR}/drivers/stm32/inc")
    
target_link_libraries(${LIB_NAME} PUBLIC free-rtos-cmsis-os)
