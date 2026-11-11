@page page_runtime_view Runtime view

@section sec_runtime_view_bahrs_sw_activity BAHRS software activity diagram

@startuml
start

:Power On;

partition "Bootloader" {
    :Start Bootloader;
    :Wait 2 seconds for STM32Cube Programmer request;

    if (Connection request received?) then (yes)
        repeat
            :Process requests from STM32CubeProgrammer;
        repeat while (Is power ON?) is (yes)

        end
    else (no)
        :Launch Target Application;
    endif
}

partition "Target Application" {
    :Initialize Peripherals;
    :Wait 2 seconds for a request from NVMManager GUI;

    if (Diagnostics request received?) then (yes)
        :Process requests from NVMManager GUI;
    endif

    :Initialize FreeRTOS;
    :Launch FreeRTOS;
    :Execute Initialization Task;

    repeat
        :Normal Operation;
    repeat while (Is power ON?) is (yes)

    end
}

@enduml

The step "Execute Initialization Task" is described in @ref sec_runtime_view_init_task.

@section sec_runtime_view_init_task Initialization task

The initialization task (`TaskInit`) prepares the whole software before any other task is allowed to run.
In one sentence: **stop all other tasks, initialize everything, start the clock, release all tasks.**

Implementation:
- V2: `TaskRoutineInit()` in `TargetApp/Rte/Src/TaskRoutines.cpp`
- V3: `RunTaskInit()` in `TargetApp_HW_V3/bahrs-v3-cube/Core/Src/task_routines.cpp`
- Stopping and releasing the tasks: class `CStartupTaskBarrier` in `shared/lib/os-call-wrappers`

@subsection subsec_runtime_view_init_task_sequence Sequence

@startuml Activity diagram: initialization task
start

note right
  Before the scheduler starts, all tasks, queues,
  semaphores and the cyclic trigger timer are created.
  The timer is not started yet.
end note

:Scheduler starts.
TaskInit runs first (highest priority);
:Suspend all other tasks
(except the idle task);
:Initialize the RTE
(creates the port mutexes);
:Initialize BSW drivers and SWCs, one after another;

if (All components initialized?) then (yes)
    :Warm up the three BAHRS filters;
    :Start the cyclic trigger timer (1 ms period);
    :Resume all suspended tasks;
    :TaskInit suspends itself for good;
    :Other tasks start running, highest priority first;
    stop
else (no)
    :Hard assertion: the device halts,
    no task is released;
    end
endif

@enduml

Notes on individual steps:
- **Suspend all other tasks.** The idle task is not suspended. FreeRTOS needs it whenever `TaskInit` waits, for example in `osDelay()`.
  The FreeRTOS timer service task is suspended as well.
- **Warm up the BAHRS filters.** Each filter contains static objects that are constructed on first use. One call of `Step(0)` constructs them inside `TaskInit`,
  before several tasks start accessing them in parallel.
- **Start the cyclic trigger timer.** The start request is only queued here. The timer service task processes it as soon as it is resumed.
- **Resume all suspended tasks.** The scheduler is locked while the tasks are resumed. All tasks therefore become ready at the same moment,
  and none of them can start before the others are resumed.

@subsection subsec_runtime_view_init_task_contracts Rules (contracts)

The sequence above is correct only if the following rules hold. Breaking a rule can let a task work on half-initialized data,
or can make the startup hang.

-# **`TaskInit` has the highest priority of all tasks.** This includes the FreeRTOS timer service task
   (`osPriorityRealtime7` for `TaskInit`, `configTIMER_TASK_PRIORITY` = 48 = `osPriorityRealtime` for the timer service task).<br>
   *Why:* `TaskInit` must run first, so that no other task can run before it is suspended.
-# **Suspending is the first action of `TaskInit`, resuming is the last.** Only `TaskInit` may do this:
   the methods of `CStartupTaskBarrier` are private and only the init routine is a friend.<br>
   *Why:* everything in between is the protected initialization phase.
-# **Code called during initialization must never wait for another task.** Waiting for an interrupt
   (for example a DMA-complete semaphore) and delays (`HAL_Delay()`, `osDelay()`) are allowed.<br>
   *Why:* all other tasks are suspended, including the timer service task. Waiting for them would block forever or time out.
   For the same reason, software timer callbacks do not run during initialization.
-# **All tasks are created before the scheduler starts, and there are at most 32 of them.**<br>
   *Why:* only tasks that exist when `TaskInit` starts are suspended, and the barrier stores at most 32 task handles.
-# **Only `TaskInit` starts the cyclic trigger timer, after successful initialization.** It must not be started in `MX_FREERTOS_Init()`.<br>
   *Why:* the timer triggers the cyclic tasks. Starting it earlier would run them before their inputs are ready.
-# **Any initialization failure ends in a hard assertion.**<br>
   *Why:* a partly initialized system must never start operating.
-# **`FreeRTOSConfig.h` enables `INCLUDE_xTaskGetIdleTaskHandle`, `configUSE_OS2_THREAD_ENUMERATE` and `configUSE_OS2_THREAD_SUSPEND_RESUME`.**<br>
   *Why:* these are needed to find, suspend and resume the tasks. `INCLUDE_xTaskGetIdleTaskHandle` is set in a `USER CODE` section,
   so CubeMX keeps it when the code is regenerated.

Good to know: interrupts stay enabled during initialization. If a sensor interrupt posts data for a task that is still suspended,
the data waits in that task's queue. The first sample processed after the release can therefore carry an older timestamp.

@section sec_runtime_view_freertos_tasks FreeRTOS tasks

@subsection subsec_runtime_view_scheduling_and_interaction Task scheduling and interactions

@subsubsection subsubsec_runtime_view_filter_chain_1 Signal chain: BAHRS Filter 1

@startuml Sequence diagram: BAHRS Filter 1 signal chain
participant "Timer Task" as TimerTask
participant "Task Receive SCAH63T Data" as Scha63TTask
participant "Task BAHRS Filter 1" as BahrsFilter1
participant "Task 10ms" as Task10ms

loop Every 5ms
    TimerTask ->> Scha63TTask: Activate

    activate Scha63TTask
    Scha63TTask -> Scha63TTask: Downsample SCHA63T data
    Scha63TTask -> Scha63TTask: Provide raw IMU data to SWC port
    Scha63TTask -> Scha63TTask: Run IMU pre-filter 1
    Scha63TTask -> Scha63TTask: Provide filtered IMU data set to BAHRS Filter 1

    alt Every 10ms
        Scha63TTask ->> BahrsFilter1: Trigger BAHRS Filter 1 iteration
        deactivate Scha63TTask
        activate BahrsFilter1
        BahrsFilter1 -> BahrsFilter1: Run BAHRS Filter 1
        BahrsFilter1 ->> Task10ms: Report completion of iteration
        deactivate BahrsFilter1
    end
end
@enduml

@subsubsection subsubsec_runtime_view_filter_chain_2_and_3 Signal chain: BAHRS Filters 2 and 3

The diagram below shows relevant task interactions for BAHRS Filters 2 and 3.

- For the BAHRS filter 2: assume X=1, Y=2
- For the BAHRS filter 3: assume X=2, Y=3

@startuml Sequence diagram: signal chain of the BAHRS Filters 2 and 3
participant "Timer Task" as TimerTask
participant "Task Poll ICM20789 Sensor X" as Icm20789Task
participant "Task BAHRS Filter Y" as BahrsFilter
participant "Task 10ms" as Task10ms
participant "ICM20789 Sensor X" as IcmSensor

activate IcmSensor

loop Every 5ms
    TimerTask ->> Icm20789Task: Activate

    activate Icm20789Task
    Icm20789Task ->> IcmSensor: Request IMU data
    deactivate Icm20789Task

    IcmSensor -->> Icm20789Task: IMU data

    activate Icm20789Task
    Icm20789Task -> Icm20789Task: Provide raw IMU data to SWC port
    Icm20789Task -> Icm20789Task: Run IMU pre-filter X
    Icm20789Task -> Icm20789Task: Provide filtered IMU data to BAHRS Filter Y

    alt Every 10ms
        Icm20789Task ->> BahrsFilter: Trigger BAHRS Filter Y iteration
        activate BahrsFilter
        BahrsFilter -> BahrsFilter: Run BAHRS Filter Y
        BahrsFilter ->> Task10ms: Report completion of iteration
        deactivate BahrsFilter
    end

    alt Every 40ms AND if NOT waiting for pressure measurement
        Icm20789Task ->> IcmSensor:  Trigger pressure measurement
        Icm20789Task -> Icm20789Task: Set "waiting for pressure" flag to true
    end

    alt "waiting for pressure" flag is true AND waiting period elapsed
        Icm20789Task ->> IcmSensor: Request pressure data
        deactivate Icm20789Task
        IcmSensor -->> Icm20789Task: Pressure data
        activate Icm20789Task
        Icm20789Task -> Icm20789Task: Provide pressure data to SWC port
        Icm20789Task -> Icm20789Task: Run pressure compensator X
        Icm20789Task -> Icm20789Task: Set "waiting for pressure" flag to false
    end

    deactivate Icm20789Task
end
@enduml

@subsubsection subsubsec_runtime_view_magnetic_heading_chain Signal chain: magnetic heading filters 1 to 3

Each of the three magnetic heading filters is fed by its own magnetometer. All three share the same safe vehicle attitude. The index X below
is 1, 2, or 3 and identifies the magnetometer, the compensator, and the magnetic heading filter of one redundant channel.

@startuml Sequence diagram: signal chain of the magnetic heading filters
participant "Task Poll Magnetometer X" as MagTask
participant "Task 10ms" as Task10ms

loop Every 100ms
    activate MagTask
    MagTask -> MagTask: Poll magnetometer
    MagTask -> MagTask: Run magnetometer compensator (hard- and soft-iron)
    MagTask -> MagTask: Transform compensated magnetic field to vehicle frame
    deactivate MagTask
end

loop Every 10ms
    activate Task10ms
    Task10ms -> Task10ms: Run attitude monitor
    Task10ms -> Task10ms: Provide Safe Attitude Data to magnetic heading filters 1-3
    Task10ms -> Task10ms: Run magnetic heading filters 1-3 (each uses its own magnetic field data)
    Task10ms -> Task10ms: Run magnetic heading monitor
    deactivate Task10ms
end
@enduml

@subsection subsec_runtime_view_task_internal_behavior Internal behavior of tasks

@subsubsection subsubsec_runtime_view_task_5_ms Cyclic task (5ms interval)

@startuml Activity diagram: 5ms task
start

:Initialize Counter to 0;

repeat
    :Wait for Semaphore;

    if (Counter == 0) then (yes)
        :Send Software Version Message;
    endif

    :Run IMU Monitor (uses filtered IMU data);
    :Transform IMU Signals to vehicle frame;
    :Send Inertial Data Message on RS232;

    if (Counter % 8 == 0) then (yes)
        :Run Barometer Monitor (uses compensated pressure data);
        :Provide Safe Pressure Data to BAHRS Filters;
    endif

    if (Counter % 20 == 0) then (yes)
        :Send Time of Inertial Data Message on RS232;
    endif

    :Increment Counter;

@enduml

@subsubsection subsubsec_runtime_view_task_10_ms Cyclic task (10ms interval)

@startuml Activity diagram: 10ms task
start

:Initialize Counter to 0;

repeat
    :Wait for all BAHRS filters to complete iteration;
    
    :Compute vehicle attitude from outputs of BAHRS filters;
    :Run attitude monitor;
    :Run vertical channel monitor;

    :Provide Safe Attitude Data to magnetic heading filters 1-3;
    :Run magnetic heading filters 1-3;
    :Run magnetic heading monitor;

    :Send Navigation Data Message on RS232;

    if (Counter % 10 == 0) then (yes)
        :Send Accuracy Data Message on RS232;
        :Send Time of Navigation Data Message on RS232;
    endif

    :Increment Counter;

@enduml

@section sec_runtime_view_internal_behavior_swc Software components 

@subsection subsec_runtime_view_imu_monitor_activity Internal behavior of IMU monitor SWC

Activity diagram of the @link CImuMonitorSwc::Run() @endlink method.

@startuml
start

:Read redundant IMU measurements from SWC ports;
:Execute Fault detection and isolation for angular rates and accelerations;
:Construct safe 6D IMU output;
:Write data to SafeImuData port;

stop

@enduml

@subsection subsec_runtime_view_barometer_monitor_activity Internal behavior of Barometer monitor SWC

Activity diagram of the @link CBaroMonitorSwc::Run() @endlink method.

@startuml
start

:Read redundant pressure measurements from SWC ports;
:Execute fault detection and isolation;

if (Is data from the Pressure Sensor 1 available?) then (yes)
    if (Is data from the Pressure Sensor 1 safe?) then (yes)
        :Provide new data to SafePressureData1 SWC port;
    else
        :Invalidate data stored in the SafePressureData1 SWC port;
    endif
else (no)
    :Invalidate data stored in the SafePressureData1 SWC port;
endif

if (Is data from the Pressure Sensor 2 available?) then (yes)
    if (Is data from the Pressure Sensor 2 safe?) then (yes)
        :Provide new data to SafePressureData2 SWC port;
    else
        :Invalidate data stored in the SafePressureData2 SWC port;
    endif
else (no)
    :Invalidate data stored in the SafePressureData2 SWC port;
endif

if (Is data from the Pressure Sensor 3 available?) then (yes)
    if (Is data from the Pressure Sensor 3 safe?) then (yes)
        :Provide new data to SafePressureData3 SWC port;
    else
        :Invalidate data stored in the SafePressureData3 SWC port;
    endif
else (no)
    :Invalidate data stored in the SafePressureData3 SWC port;
endif

stop

@enduml

@subsection subsec_runtime_view_vertical_channel_monitor_activity Internal behavior of Vertical channel monitor SWC

Activity diagram of the @link CVerticalChannelMonitorSwc::Run() @endlink method.

@startuml
start

:Read redundant vertical motion data (height and vertical speed) from SWC ports;
:Execute fault detection and isolation;

if (Are height AND vertical speed from the BAHRS Filter 1 safe?) then (no)
    if (Are height AND vertical speed from the BAHRS Filter 2 safe?) then (no)
        if (Are height AND vertical speed from the BAHRS Filter 3 safe?) then (yes)
            :Provide vertical motion data from the BAHRS Filter 3 into the SafeVerticalChannelData SWC port;
        else (no)
            :Invalidate vertical motion data in the SafeVerticalChannelData SWC port;
        endif
    else (yes)
        :Provide vertical motion data from the BAHRS Filter 2 into the SafeVerticalChannelData SWC port;
    endif
else (yes)
    :Provide vertical motion data from the BAHRS Filter 1 into the SafeVerticalChannelData SWC port;
endif

stop

@enduml

@subsection subsec_runtime_view_attitude_monitor_activityInternal behavior of  Attitude monitor SWC

Activity diagram of the @link CAttitudeMonitorSwc::Run() @endlink method.

@startuml
start

:Read redundant vehicle attitude data (roll and pitch) from SWC ports;
:Execute fault detection and isolation;

if (Are roll AND pitch from the BAHRS Filter 1 safe?) then (no)
    if (Are roll AND pitch from the BAHRS Filter 2 safe?) then (no)
        if (Are roll AND pitch from the BAHRS Filter 3 safe?) then (yes)
            :Provide attitude data from the BAHRS Filter 3 into the SafeVehicleAttitude SWC port;
        else (no)
            :Invalidate attitude in the SafeVehicleAttitude SWC port;
        endif
    else (yes)
        :Provide attitude from the BAHRS Filter 2 into the SafeVehicleAttitude SWC port;
    endif
else (yes)
    :Provide attitude from the BAHRS Filter 1 into the SafeVehicleAttitude SWC port;
endif

stop

@enduml

@subsection subsec_runtime_view_magnetic_heading_monitor_activity Internal behavior of Magnetic heading monitor SWC

Activity diagram of the @link CMagneticHeadingMonitorSwc::Run() @endlink method.

@startuml
start

:Read redundant magnetic heading data from SWC ports;
:Execute fault detection and isolation;

if (Is the heading from the magnetic heading filter 1 valid AND safe?) then (no)
    if (Is the heading from the magnetic heading filter 2 valid AND safe?) then (no)
        if (Is the heading from the magnetic heading filter 3 valid AND safe?) then (yes)
            :Provide heading from the filter 3 into the SafeMagneticHeading SWC port with health "safe";
        else (no)
            if (Is the monitor output valid?) then (yes)
                :Provide the monitor output into the SafeMagneticHeading SWC port with health "integrity risk";
            else (no)
                :Provide invalid heading with health "unavailable";
            endif
        endif
    else (yes)
        :Provide heading from the filter 2 into the SafeMagneticHeading SWC port with health "safe";
    endif
else (yes)
    :Provide heading from the filter 1 into the SafeMagneticHeading SWC port with health "safe";
endif

stop

@enduml

@subsection subsec_runtime_view_scalar_signal_fdi_activity FDI activity for an arbitrary scalar signal

@startuml

start

:Access fault detection status;

if (Detection available?) then (yes)
	if (Alarm raised?) then (no)
		:Provide signal from primary sensor;
		stop
	else (yes)
		:Check fault isolation availability;
		if (Isolation available?) then (yes)
			:Check fault isolation status;
			if (Isolation succeeded?) then (yes)
				if (Primary sensor excluded?) then (no)
					:Provide signal from primary sensor;
				else (yes)
					:Provide signal from back-up sensor;
				endif

				stop
			else (no)
				:Provide mean value;
				stop
			endif
		else (no)
			:Provide mean value;
			stop
		endif
	endif
else (no)
	:Provide the only available signal;
	stop
endif

@enduml

