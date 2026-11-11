---
name: bahrs-add-swc
description: 'Add a new Software Component (SWC) to the BAHRS application software layer. Use when creating a new ASW component, adding a FreeRTOS runnable, registering a new class in the RTE, or extending the CRTP CSoftwareComponent pattern. Covers header/source scaffolding, CMakeLists, rte.json registration, Init() wiring, and replay tool integration.'
---

# Add a New ASW Software Component (SWC)

## When to Use

- Creating a new application software component (ASW SWC)
- Adding a new runnable to the FreeRTOS task schedule
- Registering a new component in the RTE for port access
- Scaffolding the CRTP `CSoftwareComponent` boilerplate

## Key Constraints (read before starting)

- **ASW components must never call STM32 HAL APIs directly.** All hardware access goes through BSW or via RTE ports.
- **No dynamic memory allocation in production code.** No `new`/`delete`/`malloc`/`free`. Use static allocation only.
- All SWCs inherit from `CSoftwareComponent<TDerived, instanceCount>` (CRTP singleton pattern).
- Mandatory API: `Init()`, `IsInitialized()`, `getInstanceImpl(unsigned)`.
- The `Init()` call must be added to the **Initialization Task** in the target app, and (if applicable) to `BahrsReplayTool.cpp` `main()`.
- RTE friendship is granted automatically by the generator when the component is in `rte.json`. Do not add `friend` declarations manually to `CRte.h`.
- **Never hand-edit generated RTE files.** If the component needs port access, register it in `rte.json` first, then regenerate.

## Reference Files

| File | Purpose |
|---|---|
| `02_Projects/Stm32Bahrs/shared/asw/` | All ASW SWC folders |
| `02_Projects/Stm32Bahrs/shared/asw/CMakeLists.txt` | Parent CMake — add `add_subdirectory` here |
| `02_Projects/Stm32Bahrs/shared/asw/swc-imu-monitor/` | Reference SWC (single-instance pattern) |
| `02_Projects/Stm32Bahrs/shared/asw/swc-bahrs-filter/` | Reference SWC (3-instance pattern) |
| `02_Projects/Stm32Bahrs/TargetApp/Rte/rte.json` | V2 component registry |
| `02_Projects/Stm32Bahrs/TargetApp_HW_V3/bahrs-v3-rte/rte.json` | V3 component registry |
| `02_Projects/Stm32Bahrs/TargetApp/Rte/generate_rte_bahrs_v2.bat` | V2 code generation |
| `02_Projects/Stm32Bahrs/TargetApp_HW_V3/bahrs-v3-rte/generate_rte_bahrs_v3.bat` | V3 code generation |
| `02_Projects/VisualStudioToolsBahrs/shared/BahrsReplayTool.cpp` | PC replay tool — wiring Init() calls |

## Step-by-Step Procedure

### Step 1 — Determine scope

Ask (or infer):
- How many instances are needed? (Typically 1; use 3 for per-IMU SWCs like `CBahrsFilterSwc`)
- Which hardware variant(s) does it target: V2, V3, or both (shared)?
- Does it participate in PC-side replay? (i.e., does it read/write `record_by_default: true` ports or produce outputs the replay needs?)
- Which RTE ports does it read? Which does it write?

### Step 2 — Create the folder structure

Under `02_Projects/Stm32Bahrs/shared/asw/`:

```
swc-my-component/
├── inc/
│   └── CMyComponentSwc.h
├── src/
│   └── CMyComponentSwc.cpp
└── CMakeLists.txt
```

File naming follows camelCase (project convention).

### Step 3 — Write the header

Use `#ifndef` include guards (never `#pragma once`). Template for a single-instance SWC:

```cpp
/// @file CMyComponentSwc.h
/// @brief Declaration of the <name> software component class.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_MY_COMPONENT_SWC_H
#define C_MY_COMPONENT_SWC_H

#include "CRte.h"

class CMyComponentSwc : public CSoftwareComponent<CMyComponentSwc, 1U>
{
  friend class CSoftwareComponent<CMyComponentSwc, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CMyComponentSwc)
  DECLARE_MANDATORY_APIS(CMyComponentSwc)

public:
  /// @brief Run one cycle of the component.
  void Run();

protected:

private:
  CMyComponentSwc(uint32_t uInstanceIndex);
  ~CMyComponentSwc() = default;

  // private members follow naming convention: suffix _
};

#endif // C_MY_COMPONENT_SWC_H
```

For a 3-instance SWC replace `1U` with `3U` and add instance-selection logic in port access methods (see `CBahrsFilterSwc` for the pattern).

### Step 4 — Write the source

Minimal implementation skeleton:

```cpp
/// @file CMyComponentSwc.cpp
/// @brief Implementation of the <name> software component class.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CMyComponentSwc.h"

CMyComponentSwc::CMyComponentSwc(uint32_t uInstanceIndex)
{
  (void)uInstanceIndex;
}

void CMyComponentSwc::initImpl()
{
  // Perform one-time initialization here.
  // Set bIsInitialized_ = true at the end when successful.
  bIsInitialized_ = true;
}

CMyComponentSwc& CMyComponentSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CMyComponentSwc soInstance{ 0U };
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

void CMyComponentSwc::Run()
{
  if (true == bIsInitialized_)
  {
    // Component logic here.
  }
}
```

### Step 5 — Create CMakeLists.txt

Follow the pattern from existing SWC CMakeLists files:

```cmake
set(LIB_NAME swc-my-component)

add_library(${LIB_NAME} STATIC "src/CMyComponentSwc.cpp")

target_include_directories(${LIB_NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/inc")

target_link_libraries(${LIB_NAME} PUBLIC bahrs-v3-rte)
```

Adjust `target_link_libraries` if the SWC depends on additional libraries (e.g., `bahrs-sensor-fusion-library`, `swc-imu-monitor`).

### Step 6 — Register in the parent CMakeLists

Add to `02_Projects/Stm32Bahrs/shared/asw/CMakeLists.txt`:

```cmake
add_subdirectory(swc-my-component)
```

### Step 7 — Register in rte.json

Add an entry to the `"components"` array in the relevant `rte.json` file(s):

```json
{
  "class_name": "CMyComponentSwc",
  "header_file": "CMyComponentSwc.h",
  "include_in_resimulation": true
}
```

- Set `"include_in_resimulation": true` if the component should be called during PC-side replay (i.e., it processes recorded sensor inputs or produces outputs needed by downstream SWCs).
- Set `"include_in_resimulation": false` for BSW drivers and hardware-facing components.

### Step 8 — Regenerate the RTE

Run the generator for each affected hardware variant:

```
02_Projects\Stm32Bahrs\TargetApp\Rte\generate_rte_bahrs_v2.bat        (V2)
02_Projects\Stm32Bahrs\TargetApp_HW_V3\bahrs-v3-rte\generate_rte_bahrs_v3.bat  (V3)
```

This regenerates `CRte.h` with the new `friend` declaration, so the SWC can access private RTE ports.

### Step 9 — Wire Init() in the target application

In the **Initialization Task** source (under `TargetApp/` or `TargetApp_HW_V3/`), add:

```cpp
#include "CMyComponentSwc.h"
// ...
CMyComponentSwc::GetInstance().Init();
```

Place the `Init()` call after any SWC it depends on (e.g., after `CRte::GetInstance().Init()`).

Add the `Run()` call in the appropriate cyclic task (5 ms or 10 ms) consistent with the component's timing requirements. Follow the existing task sequence documented in the architecture runtime view.

### Step 10 — Wire Init() in BahrsReplayTool (if applicable)

If `"include_in_resimulation": true`, add to `02_Projects/VisualStudioToolsBahrs/shared/BahrsReplayTool.cpp` `main()`:

```cpp
#include "CMyComponentSwc.h"
// ...
CMyComponentSwc::GetInstance().Init();
```

Maintain the same Init() ordering as in the embedded target. `CRte::GetInstance().Init()` must always be first.

### Step 11 — Verify

1. **Build PC tools**: `05_Jenkins/Build_VS.bat Debug`
2. **Build embedded target**: `05_Jenkins/Build_TargetApp_STM32_V2.bat debug` (V2) or `05_Jenkins/Build_TargetApp_STM32_V3.bat debug` (V3)

## Common Mistakes

- Forgetting to call `Init()` in both the embedded target **and** `BahrsReplayTool.cpp` when `include_in_resimulation: true`.
- Setting `bIsInitialized_ = true` before initialization is actually complete, masking failures.
- Accessing RTE ports from the SWC before regenerating `CRte.h` → compiler error: port is private, `friend` declaration missing.
- Using `new`/`malloc` inside the SWC — prohibited in production code.
- Adding a `friend` declaration manually to `CRte.h` — always regenerate instead.
- Forgetting to add `add_subdirectory` to the parent `CMakeLists.txt`, causing the SWC to be silently excluded from the build.
