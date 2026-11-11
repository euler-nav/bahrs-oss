# BAHRS Project — GitHub Copilot Instructions

EULER-NAV Baro-Inertial Attitude and Heading Reference System (BAHRS).
Copyright AMS Advanced Air Mobility Sensors UG.

The device delivers attitude (roll, pitch, magnetic heading), height, vertical speed, angular rates, and specific force over RS-232 and CAN. It uses triple-redundant IMUs, barometers, and magnetometers for safety.

---

## Repository layout

```
00_Documentation/         arc42 software architecture document (Doxygen + PlantUML)
  doc_pages/              arc42-style .md pages (building_block_view, runtime_view, etc.)
  Doxyfile_BAHRS_V2       Doxygen config for hardware V2
  Doxyfile_BAHRS_V3       Doxygen config for hardware V3
  generate_documentation.bat / generate_documentation_v2.bat / generate_documentation_v3.bat
  open_documentation_v2.bat / open_documentation_v3.bat
02_Projects/
  Stm32Bahrs/             Embedded target application (STM32, FreeRTOS)
    shared/lib/           Shared libraries: common/ (CSerializable etc.), swc-ports/ (RTE port base classes), stm32-lib-common/, bahrs-sensor-fusion/ (headers + precompiled binaries); V2 sensor drivers (CBmm150Driver, CIcm20789Driver) are in TargetApp/bsw/<driver>/{inc,src}
    shared/asw/           Application software components (SWCs) — shared across V2 and V3
    TargetApp/            Hardware V2: RTE, task entry points, rte.json, generate_rte_bahrs_v2.bat
    TargetApp_HW_V3/      Hardware V3: RTE, task entry points, rte.json, generate_rte_bahrs_v3.bat
  VisualStudioToolsBahrs/
    shared/               Shared PC-side source (BahrsReplayTool.cpp, Crc32.h/.cpp, CFileDebugOutputHandler)
    BahrsReplayToolBahrsV2/   Replay tool VS project for V2 hardware
    BahrsReplayToolBahrsV3/   Replay tool VS project for V3 hardware
    DebugOutputToMatBahrsV2/  MAT exporter VS project for V2 (auto-generated)
    DebugOutputToMatBahrsV3/  MAT exporter VS project for V3 (auto-generated)
    SerialProtocolToMat/      Converts RS-232 serial log to .mat
  BahrsEvaluationTool/    MATLAB/Octave evaluation scripts, replay demo with V3 sample logs
03_Firmware/              Prebuilt firmware images (bootloader, releases, HW test app)
05_Jenkins/               Build and unit test scripts (also used by the internal CI)
06_Tools/
  rte_generator/          Python RTE code generator
    rte_generator/        rte_generator.py + 5 templates (CRte_header, CRte_cpp,
                          DebugOutputToMat_cpp, WriteToRtePort, RteTypes_header)
  NVMManager/             Python GUI for NVM image editing and upload/download
  NVM_driver_generator/   NVM driver generator
  MagCalib/               MATLAB magnetometer calibration scripts
  create_release/         Submodule-to-files conversion script (convert_submodules_to_files.py)
99_3rd_party/             Eigen, GoogleTest, TinyMAT, MathJax, Doxygen, PlantUML; open-aided-navigation and libcanard (submodules)
```

---

## Hardware variants: V2 vs V3

Two hardware variants are maintained in parallel. Each has its own:
- RTE folder with `rte.json` and a `generate_rte_bahrs_v*.bat` script
- VS replay tool project (`BahrsReplayToolBahrsV2` / `BahrsReplayToolBahrsV3`)
- VS MAT exporter project (`DebugOutputToMatBahrsV2` / `DebugOutputToMatBahrsV3`)
- STM32 build target (`TargetApp` / `TargetApp_HW_V3`)
- Doxygen config (`Doxyfile_BAHRS_V2` / `Doxyfile_BAHRS_V3`)

V3 additionally supports `ENABLE_PRINTF` (CMake option) for local debug output via `printf`.

---

## C++ coding conventions

See `Coding_guidelines_for_C_and_C_plus_plus.md` for the full rules (naming, indentation, braces, casts, include guards). Additions not covered there:

- **Doxygen style:** `///` line comments only — never `/** */` blocks. Add `/// @brief`, `/// @param`, `/// @return` on all public API declarations.
- **File header** (every `.h` and `.cpp`):
  ```
  /// @file FileName.h
  /// @brief Declaration/Implementation of ...
  /// @copyright Copyright <YEAR>. AMS Advanced Air Mobility Sensors UG. All rights reserved.
  ```
  No `@author`, no `@date`.
- **`#endif` and closing-brace comments:** `#endif // GUARD_NAME`; `} // namespace NMyNamespace` — never `/* */` style.
- **Single return per function:** wrap logic in `if (condition) { ... }` rather than using an early `return`.
- **Yoda conditions:** constant/literal on the left — `if (false == bFlag_)`.

---

## Software architecture layers

The BAHRS software is split into two binaries: **Bootloader** and **Target Application**.

The **Target Application** is composed of four layers:

```
┌─────────────────────────────────────────────────────────┐
│           Application Software (ASW)                    │
│  SWCs: IMU monitor, Baro monitor, BAHRS filters, etc.   │
│  Rule: ASW must NOT use HAL APIs directly               │
├─────────────────────────────────────────────────────────┤
│           Runtime Environment (RTE)                     │
│  Singleton CRte; mutex-protected ports; auto-generated  │
├───────────────────────┬─────────────────────────────────┤
│  Basic Software (BSW) │  Library Collection             │
│  HAL + FreeRTOS       │  Eigen, OAN fusion lib, etc.    │
│  Sensor drivers       │                                 │
│  BSW CAN use HAL APIs │                                 │
└───────────────────────┴─────────────────────────────────┘
```

The AHRS/KF algorithm core is distributed as a precompiled static library (`02_Projects/Stm32Bahrs/shared/lib/bahrs-sensor-fusion/Binaries/libBahrsSensorFusionLibrarySTM32.a`). Its internals are not in this repository and cannot be modified.

### Basic Software (BSW) components

BSW components can use STM32 HAL APIs directly. ASW components must not.

| Driver / Handler | Class |
|---|---|
| BMP384 barometer sensor driver | `CBmp384Driver` |
| BMM150 magnetometer sensor 1 driver | `CBmm150Driver` |
| BMM150 magnetometer sensor 2 driver | `CBmm150Driver` |
| SCHA63T IMU sensor driver | `CScha63TDriver` |
| ICM20789 IMU+baro sensor 1 driver | `CIcm20789Driver` |
| ICM20789 IMU+baro sensor 2 driver | `CIcm20789Driver` |
| MMC5983 magnetometer sensor driver | `CMmc5983Driver` |
| RS-232 output handler | `CRs232OutputHandler` |
| CAN bus handler | `CCanHandler` |
| CAN output handler | `CCanOutputHandler` |
| Sync pulse handler | `CSyncPulseHandler` |

---

## RTE (Runtime Environment)

### Design principles

- `CRte` is a singleton; access via `CRte::GetInstance()`.
- All inter-SWC data exchange is through `CSoftwareComponentPort<T, PortId>` — mutex-protected, templated.
- Port IDs are defined in `CRte::EPortIds` (enum class, `uint8_t`).
- SWC ports are **private members** of `CRte`. Only `friend` classes/functions declared in the generated `CRte.h` may access them.
- Port IDs **must be in ascending order** when passed to `CPortReader::ReadPorts()` — enforced at compile time via `TArePortIdsAscending`.
- **Port data types must inherit `NLibCommon::CSerializable`** and implement `ToByteVector()` and `FromByteVector()`. This is mandatory for binary log recording and PC-side replay.

### RTE code generation workflow

**Never hand-edit generated files** (CRte.h, CRte.cpp, RteTypesGenerated.h, WriteToRtePort.cpp, DebugOutputToMat.cpp) without also updating the corresponding template.

To add/change/remove a port or component:
1. Edit `rte.json` (in `TargetApp/Rte/` for V2, or `TargetApp_HW_V3/bahrs-v3-rte/` for V3).
2. Run the corresponding batch script:
   - V2: `TargetApp/Rte/generate_rte_bahrs_v2.bat`
   - V3: `TargetApp_HW_V3/bahrs-v3-rte/generate_rte_bahrs_v3.bat`
3. The script calls `rte_generator.py` with paths to the RTE folder, DebugOutputToMat project, and BahrsReplayTool project.

Generator templates (in `06_Tools/rte_generator/rte_generator/`):
- `CRte_header_template.h` → `CRte.h`
- `CRte_cpp_template.cpp` → `CRte.cpp`
- `RteTypes_header_template.h` → `RteTypesGenerated.h`
- `WriteToRtePort_template.cpp` → `WriteToRtePort.cpp`
- `DebugOutputToMat_cpp_template.cpp` → `DebugOutputToMat.cpp`

The checked-in batch wrappers call `06_Tools/rte_generator/rte_generator/rte_generator.py` and pass three concrete paths:
- the local RTE directory (`--rte-path .`)
- the matching `DebugOutputToMatBahrsV2` or `DebugOutputToMatBahrsV3` project (`--debug-output-path ...`)
- the matching `BahrsReplayToolBahrsV2` or `BahrsReplayToolBahrsV3` project (`--replay-tool-path ...`)

### SWC pattern

All SWCs inherit from `CSoftwareComponent<TDerived, instanceCount>` (CRTP singleton).
Mandatory API: `Init()`, `IsInitialized()`, `getInstanceImpl(unsigned)`.

---

## Application software components (SWCs)

| SWC | Folder (`shared/asw/`) | Purpose |
|-----|----------|---------|
| `CBahrsFilterSwc` | `swc-bahrs-filter` | KF-based AHRS filter (3 independent instances) |
| `CImuMonitorSwc` | `swc-imu-monitor` | IMU FDI (fault detection and isolation) |
| `CBaroMonitorSwc` | `swc-barometer-monitor` | Barometer consistency monitoring |
| `CAttitudeMonitorSwc` | `swc-attitude-monitor` | Cross-checks 3 filter attitude outputs |
| `CVerticalChannelMonitorSwc` | `swc-vertical-channel-monitor` | Safe vertical channel output |
| `COutputTransformer` | `swc-output-transformer` | Transforms filter output to vehicle frame; **must be Init()'d before use** |
| `CMagnetometerCompensator` | `swc-magnetometer-compensator` | Hard/soft iron compensation |
| `CMagneticHeadingFilterSwc` | `swc-magnetic-heading-filter` | Magnetic heading filter (3 independent instances, one per magnetometer) |
| `CMagneticHeadingMonitorSwc` | `swc-magnetic-heading-monitor` | Cross-checks 3 magnetic heading filter outputs, provides safe heading with health |
| `CImuPreFilterSwc` | `swc-imu-pre-filter` | IMU signal pre-filtering |
| `CPressureCompensator` | `swc-pressure-compensator` | Pressure signal compensation |

---

## FreeRTOS task scheduling (runtime view)

The OS is FreeRTOS with CMSIS-OS v2 API. All OS objects are statically allocated **except mutexes** (CMSIS-OS v2 limitation — mutexes are created dynamically at init).

### 5 ms cyclic task

1. (Counter == 0) Send Software Version Message
2. Run IMU Monitor (`CImuMonitorSwc::Run()`)
3. Transform IMU signals to vehicle frame (`COutputTransformer::TransformImuSignals()`)
4. Send Inertial Data Message on RS-232
5. (Counter % 8 == 0 → 25 Hz) Run Barometer Monitor; Provide Safe Pressure Data to BAHRS Filters
6. (Counter % 20 == 0) Send Time of Inertial Data Message on RS-232

### 10 ms cyclic task

1. Wait for **all 3 BAHRS filters** to complete iteration (event flags)
2. Compute vehicle attitude from filter outputs (`COutputTransformer::TransformOrientation()`)
3. Run Attitude Monitor (`CAttitudeMonitorSwc::Run()`)
4. Run Vertical Channel Monitor (`CVerticalChannelMonitorSwc::Run()`)
5. Provide Safe Attitude Data to magnetic heading filter
6. Run Magnetic Heading Filter
7. Send Navigation Data Message on RS-232
8. (Counter % 10 == 0) Send Accuracy + Time of Navigation Data Messages

### Startup sequence (Target Application)

1. Initialize peripherals
2. Wait 2 seconds for NVMManager GUI diagnostics request
3. Initialize and launch FreeRTOS
4. Execute Initialization Task (all SWCs and BSW components call `Init()`)
5. Normal operation (all cyclic tasks running)

---

## Dynamic memory rules

**Production code must not use `new`/`delete`/`malloc`/`free`.** The system enforces this in four layers:

| Layer | Mechanism |
|---|---|
| 1 — Disable newlib heap | `_sbrk()` replaced with assertion stub |
| 2 — Block C allocators | `malloc`/`free`/`calloc`/`realloc` stubs assert in production; in debug (V3 + `ENABLE_PRINTF`) `_malloc_r`/`_free_r` are routed through C++ operators |
| 3 — Route C++ new/delete to FreeRTOS | All `operator new`/`delete` overloads call `pvPortMalloc`/`vPortFree` |
| 4 — Prevent allocation in interrupts | FreeRTOS heap functions assert when called from ISR context |

**Permitted exceptions:**
- `std::vector` in debug serialization (RS-232 transmission, debug builds only)
- `printf` via `ENABLE_PRINTF` (V3 debug builds only, routes through FreeRTOS heap)
- FreeRTOS mutexes (created dynamically at init via CMSIS-OS v2; all other FreeRTOS objects are static)

---

## PC replay tool (`BahrsReplayTool.cpp`)

Replays a recorded `.bin` log by re-running all ASW SWCs on the PC.

**Calibration:** in PC (`_MSC_VER`) builds, `CMagnetometerCompensator` and `CPressureCompensator` use hard-coded values that match the two sample logs in `02_Projects/BahrsEvaluationTool/data/`, not the device NVM. Update them (e.g. from `06_Tools/MagCalib/mag_calib.m`) when replaying other recordings.

**Critical:** `COutputTransformer::GetInstance().Init()` **must be called** in `main()` after `CRte::GetInstance().Init()` — otherwise `bIsInitialized_` stays false, `TransformOrientation()` writes invalid attitude data to the vehicle attitude ports, and the attitude monitor outputs unavailable/invalid data for the entire replay.

Init order in `main()`:
```cpp
CRte::GetInstance().Init();
COutputTransformer::GetInstance().Init();   // ← mandatory, do not omit
// ... other SWC inits
```

---

## Build system

### STM32 (embedded target)
Built with CMake + Ninja and the GNU Tools for STM32 from STM32CubeIDE 1.18.0. The variant argument is required and lowercase (`debug`, `release`, `debug-with-debug-output`); an optional second argument `dronecan` selects the DroneCAN stack:
- V2: `05_Jenkins/Build_TargetApp_STM32_V2.bat <debug|release|debug-with-debug-output> [dronecan]`
- V3: `05_Jenkins/Build_TargetApp_STM32_V3.bat <debug|release|debug-with-debug-output> [dronecan]`

### Visual Studio tools (PC side)
Built with MSBuild:
- `05_Jenkins/Build_VS.bat <Debug|Release>`
- Solution: `02_Projects/VisualStudioToolsBahrs/VisualStudioToolsBahrs.sln`

### Unit tests
- `05_Jenkins/Unit_Tests.bat <Debug|Release>` (run from the repository root after building the VS solution)

### Documentation (Doxygen + PlantUML)
- Java must be installed and on `PATH` (required for PlantUML).
- `00_Documentation/doc_pages/` is the source of truth for architecture content; inspect these pages before editing architecture documentation or summarizing the design.
- Generate: `00_Documentation/generate_documentation_v2.bat` / `generate_documentation_v3.bat`
- Wrapper script: `00_Documentation/generate_documentation.bat`
- Open output: `00_Documentation/open_documentation_v2.bat` / `open_documentation_v3.bat`
- The `doxygen.exe` binary is distributed with the repository in `99_3rd_party/doxygen/`.

---

## Tooling workflows

### NVM tooling
- `06_Tools/NVM_driver_generator/nvm_driver_generator.py` — generates NVM driver header from `nvm_map.json`.
- `06_Tools/NVMManager/` — Python 3.11 + Pipenv GUI for NVM image editing and device upload/download.

### Other tools
- `06_Tools/MagCalib/` — MATLAB magnetometer calibration scripts.
- `06_Tools/create_release/convert_submodules_to_files.py` — replaces submodules matching a URL pattern (default: `euler-nav-component-library`) with regular files; `--source-clone <path> [--checkout]` copies and verifies the pinned files from another clone. Stages only, never commits.

---

## Key enumerations and data types

### `CSerialProtocol::ESignalHealthInfo` (in `eHealth_` fields)
```
0 = eUnavailable
1 = eSafe
2 = eIntegrityRisk
```

### `EDetectionResult` / `EIsolationResult` (IMU monitor, CommonMonitorTypes.h)
```
0 = invalid
1 = unavailable
2 = good
3 = failure / failed
```

### `CBahrsFilterOutput::eFilterMode_`
```
0 = initializing
1 = idle
2 = running (navigation mode) ← plot only this mode in filter state plots
```

### Key port data structures

| MAT variable name | C++ struct | Notable fields |
|---|---|---|
| `PortBahrsFilterOutput1/2/3` | `CBahrsFilterOutput` | `oState_`, `oStateStd_`, `eFilterMode_`, `uTimestampUs_` |
| `PortImuDataAfterMonitor` | `CImuDataAfterMonitor` | `oSpecificForceX/Y/Z_`, `oAngularRateX/Y/Z_` each with `fSignal_`, `eDetectionResults_`, `eIsolationResults_`, `uTimestampUs_` |
| `PortSafePressureData1/2/3` | `SBarometerMeasurement` | `fPressure_`, `fTemperature_`, `uTimestampUs_`, `uValid_` |
| `PortSafeAttitudeData` | `SSafeAttitudeData` | `fRoll_` [rad], `fPitch_` [rad], `fAttitudeStd1_` [rad], `fAttitudeStd2_` [rad], `uTimestampUs_`, `eHealth_` |
| `PortSafeVerticalChannelData` | `SSafeVerticalChannelData` | `fHeight_` [m], `fVelocityDown_` [m/s], `uTimestampUs_`, `eHealth_` |

### Quaternion convention
`oQuaternionBodyToNed_` is `[W, X, Y, Z]` (scalar first).
`lib_quatToEulerV(q)`: input `4×N [W;X;Y;Z]`, output `3×N [roll;pitch;heading]` in radians.

---

## Shared PC-side utilities

- **`Crc32.h/.cpp`** (`shared/`): single canonical CRC-32 (MPEG-2) implementation. Include this everywhere; **never copy-paste** the function into individual `.cpp` files.
- **`CFileDebugOutputHandler`**: builds and writes the binary debug log. Uses `Crc32()`.
- All VS projects must include `shared/Crc32.cpp` and `shared/Crc32.h` in their `.vcxproj`.

---

## MATLAB/Octave evaluation scripts

Location: `02_Projects/BahrsEvaluationTool/`

### Variable naming
- Use `accel` (not `acc`) and `gyro` (not `gyr`) in all MATLAB variable names.

### Figure title helper
Use the standalone `localSgtitle(str)` function (in `localSgtitle.m`).
- **Do not** re-implement `localSgtitle` as a local function inside each script.
- **Do not** call MATLAB's `sgtitle` directly — `localSgtitle.m` uses a hidden-axes text annotation for Octave compatibility.

### Legend location
Always use `'southoutside'`, `'northeast'`, or another explicit location.
**Never use `'best'`** — it is not supported in Octave and generates a warning.

### Health marker convention (all monitor plots)
```
blue  *  (b*)  = safe          (eHealth_ == 1)
yellow □  (ys) = unavailable   (eHealth_ == 0)
red   x  (rx) = integrity risk (eHealth_ == 2)
```
Use nan-placeholder `plot(nan, nan, ...)` calls to keep legend entry count stable when a mask is empty.

### Plot function inventory

| Function | Figure layout | Data source |
|---|---|---|
| `plotBahrsFilterState(filterData, name)` | 3 figures: 5×1 (vert+attitude), 6×2 (biases+scales), 3×1 (accel NED) | `PortBahrsFilterOutput1/2/3` |
| `plotImuMonitorResults(imuData, name)` | 1 figure, 6×2 (signal+health left, detection+isolation dots right) | `PortImuDataAfterMonitor` |
| `plotBarometerMonitorResults(p1,p2,p3, name)` | 1 figure, 2×1 (pressure, temperature) | `PortSafePressureData1/2/3` |
| `plotSafeAttitudeData(attData, name)` | 1 figure, 3×1 (roll±3σ, pitch±3σ, health dot) | `PortSafeAttitudeData` |
| `plotSafeVerticalChannelData(vcData, name)` | 1 figure, 3×1 (height, vel-down, health dot) | `PortSafeVerticalChannelData` |

### Master run script
`script_runBahrsFilterReplay.m` — orchestrates replay executable → MAT export → all plots.
Config flags: `testConfig.plotBahrsFilter1/2/3`, `plotImuMonitor`, `plotBarometerMonitor`, `plotAttitudeMonitor`, `plotVerticalChannelMonitor`.
All plot calls are guarded with `isfield(matData, 'Port...')` AND the config flag.

---

## Software architecture documentation

`00_Documentation/` contains an arc42-style document (12 chapters) rendered via Doxygen with PlantUML. Source pages are in `doc_pages/` — inspect them before editing or summarizing the design. When writing, maintain the arc42 chapter structure and use Doxygen `@page`/`@section`/`@startuml` syntax consistent with existing pages.

---

## Common mistakes to avoid

1. **Missing `COutputTransformer::Init()` in replay tool** — see PC replay tool section above.
2. **Copying `Crc32` into new files** — always `#include "Crc32.h"` from `shared/`.
3. **Using `#pragma once`** — use `#ifndef` guards.
4. **Using `'Location', 'best'` in MATLAB `legend()`** — use an explicit location.
5. **Using `sgtitle()` directly** — use `localSgtitle()`.
6. **Writing `\pm` or `\sigma` in `sprintf` format strings** — must be escaped as `\\pm` and `\\sigma` to survive `sprintf` and reach the TeX renderer.
7. **Updating generated files without updating the generator template** — `DebugOutputToMat.cpp`, `CRte.h`, `CRte.cpp`, `RteTypesGenerated.h`, and `WriteToRtePort.cpp` must be changed via their generator templates or JSON config, then regenerated.
8. **Adding new RTE ports without regenerating** — after changing `rte.json`, run `generate_rte_bahrs_v2.bat` or `generate_rte_bahrs_v3.bat` so both the RTE and matching PC-side generated files stay in sync.
9. **Adding dynamic memory allocation in production code** — prohibited; use static allocation. The only permitted exceptions are mutexes (FreeRTOS heap at init) and debug serialization (`std::vector` in debug builds).
10. **ASW using HAL APIs directly** — only BSW components are allowed to use STM32 HAL APIs.
11. **Port data type not implementing `CSerializable`** — every new port data type must inherit `NLibCommon::CSerializable` and implement `ToByteVector()` / `FromByteVector()`.
12. **Forgetting `git submodule update --init --recursive`** after a fresh clone — the open-aided-navigation library and libcanard (DroneCAN) will be missing.
13. **Using `/** */` Doxygen comment blocks** — use `///` line comments only.
14. **`#endif /* GUARD */` or `} /* namespace */`** — use `#endif // GUARD` and `} // namespace NMyNamespace` instead.
15. **Early returns** — prefer `if (condition) { ... }` over `return` mid-function to keep a single implicit return.
16. **`@author` / `@date` in file headers** — use `@copyright Copyright <YEAR>. AMS Advanced Air Mobility Sensors UG. All rights reserved.` instead.
