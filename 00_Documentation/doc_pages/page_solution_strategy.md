@page page_solution_strategy Solution strategy

@section sec_solution_strategy_general General

@subsection subsec_solution_strategy_operating_system Operating system

The FreeRTOS operating system with CMSIS API is used.

**Rationale 1**: we need multiple tasks that are executed at different frequencies.

**Rationale 2**: we use FreeRTOS because

1. It is free, open source, and extremely lightweigh
2. STM32CubeIDE has a built-in support for FreeRTOS configuration

**Rationale 3**: we use CMSIS API v2 because

1. STM32CubeIDE relies on it
2. The API offers an option to switch to a different RTOS in future, for example to some safe RTOS

@subsection subsec_solution_strategy_programming_language Programming language

The software is written in C and C++. The languages can be mixed.

**Rationale**: C++ features are handy for implementation of sensor fusion algorithms, at the same time HAL libraries and FreeRTOS source files are in C.

@subsection subsec_solution_strategy_source_file_types Source code classification

All the source code files shall fall into one of the following categories:

-# Software components
-# Operating system
-# Libraries

@subsubsection subsubsec_solution_strategy_software_component Software components

A software component is a class that realizes one or more primary or auxiliary system functions mapped to the software. A wrapper that inplements a C-API of a software component belongs to the software component.

@subsubsection subsubsec_solution_strategy_operating_system Operating system

All the operating system source files including API wrappers belong to the operating system.

@subsubsection subsubsec_solution_strategy_libraries Libraries

Everything that is neither a software component nor a piece of an operating system. Examples: linear algebra library, runtime environment (RTE) implementation, STM32 HAL libraries.

@subsection subsubsec_solution_strategy_dynamic_memory Usage of dynamic memory

Dynamic memory allocation is **prohibited** in production code to avoid non-deterministic timing, heap fragmentation, and memory exhaustion risks critical for safety-critical systems.

@subsubsection subsubsec_solution_strategy_dynamic_memory_exceptions Exceptions

Dynamic memory is permitted only in the following scenarios:

-# **Debug builds only**: Debug output serialization (e.g., `std::vector` for RS232 transmission) and printf functionality when explicitly enabled via CMake option `ENABLE_PRINTF`. These features must never appear in release builds.
-# **FreeRTOS mutexes**: CMSIS-OS v2 API does not provide static mutex allocation, so mutexes are created dynamically during initialization. All other FreeRTOS objects (tasks, queues, semaphores) are allocated statically.

@subsubsection subsubsec_solution_strategy_dynamic_memory_implementation Implementation

The system uses a multi-layered approach to prevent accidental dynamic memory usage:

**Layer 1: Disable newlib heap**

The `_sbrk()` function (foundation of C standard library allocators) is replaced with a stub that triggers an assertion, preventing initialization of newlib's heap.

**Layer 2: Block C memory functions**

Standard C memory functions (`malloc`, `free`, `calloc`, `realloc`) are replaced with stubs that assert immediately in production builds.

When `ENABLE_PRINTF` is defined for local debugging (V3 only, via CMake option), the `_malloc_r()` and `_free_r()` are wrapped and routed through C++ operators. The `_calloc_r()` and `_realloc_r()` remain unsupported as printf does not require them.

**Layer 3: Route C++ new/delete to FreeRTOS**

All C++ `operator new` and `operator delete` overloads are redirected to FreeRTOS heap functions (`pvPortMalloc`, `vPortFree`). **Note**: These operators remain functional in release builds but are prohibited by coding guidelines and enforced through code reviews. This provides:
-# Single heap for all dynamic allocations (debug builds and FreeRTOS mutexes)
-# Centralized memory tracking
-# Consistent out-of-memory handling (assertion on allocation failure)

**Layer 4: Prevent allocation in interrupts**

All allocation functions assert if called from interrupt context, as FreeRTOS heap functions are not interrupt-safe.

**Layer 5: No aligned allocation support**

FreeRTOS heap does not support memory alignment. Calls to aligned `new`/`delete` operators trigger assertions.

@subsubsection subsubsec_solution_strategy_dynamic_memory_guidelines Developer guidelines

**Allowed in debug builds only**:
- Standard containers: `std::vector`, `std::string`, etc. (wrapped in `#ifdef SEND_DEBUG_OUTPUT`)
- Printf-family functions (when `ENABLE_PRINTF` CMake option is enabled)

**Forbidden in all builds**:
- C-style allocation: `malloc()`, `calloc()`, etc.
- Dynamic allocation in production code paths
- Dynamic allocation in interrupt handlers
- Aligned memory allocation

**Rationale**: This approach provides multiple safety barriers while maintaining flexibility for development and debugging. Violations are enforced through a combination of hard assertions (C allocation functions, aligned allocations, interrupt context usage) and policy-based code reviews (new/delete operators).

@section sec_solution_strategy_patterns_and_principles Patterns and principles

@subsection subsec_solution_strategy_software_components Software components

@subsubsection subsubsec_solution_strategy_swc_principles Principles

-# SWCs shall exchange data by means of software component ports provided by RTE
-# SWCs are split into two categories: basic software components and application software components.
-# Application software components shall not access functions from HAL libraries.
-# Application software components shall not access OS APIs.
-# A software component shall be implemented as a class.
-# SWC instances shall be allocated on static memory during system initialization.
-# SWCs shall not be created dynamically.
-# SWCs shall not be copied of moved.
-# One class can implement multiple similar SWCs. Example: a driver of a sensor that is present twice in the system.
-# The following APIs are mandatory for all software components
    - Initialization
    - Initialization status accessor
    - Instance getter
-# Every SWC class shall provide a number of available SWC instances

@subsubsection subsubsec_solution_strategy_swc_patterns Design patterns

-# All software components shall inherit from the interface class template CSoftwareComponent following the Curiously Recurring Template Pattern (CRTP).
-# The base class CSoftwareComponent shall delete copy and move constructors and assignment operators
-# A constructor and destructor of the base class CSoftwareComponent shall be private
-# SWC instances shall be declared as static local variables in the implementation of the static API CSoftwareComponent::GetInstance.

**Remark**: the aforementioned design patterns shall help to fulfill the software component design principles listed in the previous section.

**Example of a SWC declaration**: CSoftwareComponentExample.

@subsection subsec_solution_strategy_runtime_environment Runtime environment (RTE)

Runtime environment (RTE) is an object that aggregates all SWC ports.

@subsubsection subsubsec_solution_strategy_rte_principles Principles and patterns

-# RTE class shall be a singleton
-# SWC ports shall be accessible only by software components
-# SWC ports shall allow thread-safe data exchange between SWCs
-# SWC ports shall be read-write accessible

@subsubsection subsubsec_solution_strategy_swc_ports Software component ports

Ports are the only data exchange mechanism between SWCs. Each port is a mutex-protected value with a unique ID and fixed data type.
Ports are created statically, generated by the RTE generator, and initialized once during system startup.
RTE generation supports timeout-based auto-invalidation configured in `rte.json`: a global `default_port_invalidation_timeout_ms` and optional per-port `invalidation_timeout_ms` override (`0` disables invalidation for that port).
An invalidation is treated like a write of default (invalid) data: it restarts the timeout, so a stale port is invalidated once per timeout period rather than on every read, and in debug-output builds it is recorded as a regular port write event, so binary logs show it and replay reproduces it without tool changes.
All port IDs must be unique across the system, and each port object is a unique C++ type because the ID is a template parameter.
Ports provide safe data exchange between SWCs that run in different FreeRTOS tasks and at different rates.
The mutex is created in @ref CSoftwareComponentPortBase::CSoftwareComponentPortBase; @ref CSoftwareComponentPortBase::Init is kept for API compatibility and only returns the readiness flag.

**Port Read() operation**

-# Locks the port mutex, applies timeout-based invalidation (if enabled), copies the current port value to the caller, and unlocks the port.
-# Returns false if the port is not ready (mutex creation failed in @ref CSoftwareComponentPortBase::CSoftwareComponentPortBase) or the mutex cannot be acquired within the short, fixed timeout (@ref CSoftwareComponentPortBase::skuPortLockTimeout).

**Port Write() operation**

-# Locks the port mutex, writes the new value, and unlocks the port.
-# Returns false if the port is not ready (mutex creation failed in @ref CSoftwareComponentPortBase::CSoftwareComponentPortBase) or the mutex cannot be acquired within the short, fixed timeout (@ref CSoftwareComponentPortBase::skuPortLockTimeout).
-# When debug output is enabled and port recording is turned on, the port content is serialized and sent over RS232.

**Bulk read of multiple ports**

`CPortReader::ReadPorts()` reads several ports as one operation to provide a consistent snapshot for a runnable.
Use this API whenever a runnable depends on multiple input ports; use single-port Read() only for isolated reads where cross-port consistency is not required.
The function is a variadic template and returns a tuple of the exact port data types, providing compile-time type safety.

-# Ports are locked strictly in ascending port ID order to avoid deadlocks. This order is enforced at compile time by a static assertion.
-# The lock attempt for each port uses the short, fixed timeout (@ref CSoftwareComponentPortBase::skuPortLockTimeout).
-# If any lock fails, no data is returned and all already-locked ports are released.
-# If all locks succeed, a tuple of port values is returned and then all ports are unlocked.
-# When debug output is enabled, a runnable-call event is logged with the runnable ID and lock status.

This bulk-read pattern is used by both target firmware and PC re-simulation. The runnable-call event and the per-write port data (when enabled) ensure the replay tool consumes the same port values as the firmware at each runnable call in real time. This keeps offline analysis aligned with in-target behavior without exposing OS or HAL APIs to application SWCs.

@section sec_solution_strategy_algorithms Signal processing algorithms

@subsection subsec_solution_strategy_attitude_and_vertical_motion_estimation Estimation of attitude and vertical motion

@subsubsection subsubsec_solution_strategy_estimation_intro Introduction

Attitude and vertical motion (including altitude and vertical velocity) are estimated using data from a 6D IMU and a barometer.
A **Kalman filter** is employed to fuse the signals from both sensors. This approach represents the current state-of-the-art and offers several key advantages:

-# Smoothing of measurement noise from the original sensor data.
-# The ability to extrapolate vertical velocity, altitude, and attitude between lower-frequency barometer readings.
-# Built-in accuracy assessment through the use of a state covariance matrix.
-# The capability to estimate inertial sensor errors and mitigate error growth during dead-reckoning.

The estimator is implement in the form of a **closed-loop error state Kalman filter**. In this formulation, a linear Kalman filter is used to estimate the error
in the state accumulated during the nonlinear state prediction process. This estimated state error is then applied to correct the predicted state, allowing the nonlinear prediction
process to continue from the corrected value.

@subsubsection subsubsec_solution_strategy_estimation_state_vector State vector

The state vector has dimension 12 and includes

1. A quaternion @f( \breve{\mathbf{q}}_{ns} @f) defining a rotation from the IMU sensor frame to the local levelled coordinate frame
2. Barometric altitude @f( h @f) and velocity in downwards direction @f( v_d @f)
3. A vector of accelerometer biases @f( \mathbf{b}_a @f)
4. A vector of gyroscope biases @f( \mathbf{b}_g @f)
5. A vector of accelerometer scale factors @f( \mathbf{s}_a @f)
6. A vector of gyroscope scale factors @f( \mathbf{s}_{g} @f)
7. A 3D acceleration vector @f( \mathbf{a}_{n} @f)

@subsubsection subsubsec_solution_strategy_estimation_error_state_vector State error vector

The state error vector has dimension 10 and includes

1. Two attitude errors @f( \psi_{n\tilde{n}_1} @f) and @f( \psi_{n\tilde{n}_2} @f)
2. Vertical channel errors @f( \delta h @f) and @f( \delta v_d @f)
3. Error of the accelerometer bias estimate @f( \delta\mathbf{b}_a @f)
4. Error of the gyroscope bias estimate @f( \delta\mathbf{b}_g @f)
5. Error of the accelerometer scale factor estimate @f( \delta\mathbf{s}_a @f)
6. Error of the gyroscope scale factors estimate @f( \delta\mathbf{s}_{g} @f)
7. Error of the 3D acceleration estimate @f( \delta\mathbf{a}_{n} @f)

@subsubsection subsubsec_solution_strategy_estimation_decisions Technology decisions

@paragraph par_solution_strategy_technology_decision_1 Decoupling of magnetic heading estimation

Magnetic heading is estimated separately from attitude and vertical motion.

**Rationale:**

Magnetometer signals are highly susceptible to environmental disturbances that vary significantly by location. These disturbances can degrade
the performance and stability of a Kalman filter, reducing the accuracy and reliability of the estimated attitude and
vertical motion parameters. Additionally, we have no control over where customers install EULER-NAV devices within the target vehicle, and
they may choose a location prone to intense magnetic interference. Because our device cannot predict or compensate for these disturbances,
incorporating magnetometer data into the core attitude and vertical motion estimates could lead to corrupted results, potentially giving customers
a false impression of our product’s performance and quality.

Although decoupled from the attitude and vertical-motion filter, magnetic heading uses the same redundancy concept as the other signals:
three independent magnetic heading filters, each fed by its own magnetometer, run in parallel and their outputs are cross-checked by the magnetic
heading monitor. The filters use the monitored safe vehicle attitude as common input; an attitude failure is covered by the attitude monitor.
The monitor output carries the integrity status of the magnetic heading that is reported to the customer.

@paragraph par_solution_strategy_technology_decision_2 Single IMU signal source for every filter

**Rationale:**

Allowing the Kalman filter to switch IMU input sources would necessitate re-initializing the estimation process for inertial sensor biases
each time a switch occurs. Frequent re-initializations -- such as those triggered by IMU monitor's false alarms -- could degrade output accuracy,
reduce filter stability, and endanger output integrity.
