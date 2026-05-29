set(LIB_NAME free-rtos-cmsis-os)

add_library(${LIB_NAME} STATIC "Source/croutine.c"
	"Source/croutine.c"
	"Source/event_groups.c"
	"Source/list.c"
	"Source/queue.c"
	"Source/stream_buffer.c"
	"Source/tasks.c"
	"Source/timers.c"
	"Source/CMSIS_RTOS_V2/cmsis_os2.c"
	"Source/portable/GCC/ARM_CM4F/port.c"
	"Source/portable/MemMang/heap_4.c")

target_include_directories(${LIB_NAME} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/CMSIS_RTOS_V2"
    "${CMAKE_CURRENT_SOURCE_DIR}/Source/portable/GCC/ARM_CM4F"
    "${CMAKE_SOURCE_DIR}/bahrs-v3-cube/Core/Inc")
    
target_link_libraries(${LIB_NAME} PUBLIC cmsis-stm32f4 hal-stm32f4)
