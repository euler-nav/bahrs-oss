---
name: bahrs-add-rte-port
description: 'Add a new port to the BAHRS Runtime Environment (RTE). Use when adding an RTE port, SWC port, inter-SWC data channel, new port data type, or modifying rte.json, including port auto-invalidation (invalidation_timeout_ms, default_port_invalidation_timeout_ms, stale port data). Covers port data type design, JSON edit, invalidation timeout choice, CSerializable implementation, code generation, and build verification for both V2 and V3 hardware variants.'
---

# Add a New RTE Port

## When to Use

- Adding a new inter-SWC data exchange channel
- Introducing a new port data type (generated or user-defined)
- Changing `record_by_default` status of an existing port
- Setting or changing the auto-invalidation timeout of a port (`invalidation_timeout_ms`) or of all ports (`default_port_invalidation_timeout_ms`)

## Key Constraints (read before starting)

- **Port ordering is a binary compatibility contract.** Port IDs are assigned by position in the `"ports"` array. The PC-side replay tool identifies ports by numeric ID — inserting a port anywhere except the end will invalidate existing `.bin` log files and break replay. **Always append new ports at the end of the `"ports"` array.**
- Port data types must inherit `NLibCommon::CSerializable` and implement `ToByteVector()` / `FromByteVector()` wrapped in `#if defined(SEND_DEBUG_OUTPUT) || defined(_MSC_VER)`.
- Types must use only fixed-width integer types (`uint8_t`, `int32_t`, `float`, `double`, `uint64_t`, …). **Never use `bool`** — its size is implementation-defined and differs between embedded and PC targets.
- Never hand-edit generated files (`CRte.h`, `CRte.cpp`, `RteTypesGenerated.h`, `WriteToRtePort.cpp`, `DebugOutputToMat.cpp`). All changes go through the JSON + template workflow.
- **Ports auto-invalidate by default.** Both `rte.json` files set `"default_port_invalidation_timeout_ms": 1000`, so a new port without `invalidation_timeout_ms` is reset to its **default-constructed value** (`T{}`) when it has not been written for more than 1000 ms. The default value of the port data type must therefore mean "invalid / unavailable", and ports that are written rarely, on events, or only once must opt out. See [Step 3a](#step-3a--choose-the-invalidation-timeout).

## Reference Files

| File | Purpose |
|---|---|
| `02_Projects/Stm32Bahrs/TargetApp/Rte/rte.json` | V2 port/component/type definitions (canonical) |
| `02_Projects/Stm32Bahrs/TargetApp_HW_V3/bahrs-v3-rte/rte.json` | V3 port/component/type definitions |
| `02_Projects/Stm32Bahrs/TargetApp/Rte/Inc/RteTypesUserDefined.h` | User-defined port types (V2) |
| `02_Projects/Stm32Bahrs/TargetApp_HW_V3/bahrs-v3-rte/Inc/RteTypesUserDefined.h` | User-defined port types (V3) |
| `02_Projects/Stm32Bahrs/TargetApp/Rte/generate_rte_bahrs_v2.bat` | V2 code generation script |
| `02_Projects/Stm32Bahrs/TargetApp_HW_V3/bahrs-v3-rte/generate_rte_bahrs_v3.bat` | V3 code generation script |
| `06_Tools/rte_generator/rte_generator/CRte_header_template.h` | Template for CRte.h |
| `06_Tools/rte_generator/rte_generator/RteTypes_header_template.h` | Template for RteTypesGenerated.h |
| `02_Projects/Stm32Bahrs/shared/lib/common/CSerializable.h` | CSerializable base class |
| `02_Projects/Stm32Bahrs/shared/lib/common/DataSerializer.h` | Serialize / Deserialize helpers |
| `02_Projects/Stm32Bahrs/shared/lib/swc-ports/CSoftwareComponentPort.h` | Port template; `uInvalidationTimeoutMs` parameter and invalidation logic |
| `06_Tools/rte_generator/rte_generator/rte_generator.py` | Timeout resolution (`get_port_invalidation_timeout_ms`) and validation (`check_rte_configuration`) |
| `02_Projects/VisualStudioToolsBahrs/BahrsUT/CSoftwareComponentTest.cpp` | Unit tests `CSoftwareComponentPortAutoInvalidation.*` (reference for exact behavior) |

## Step-by-Step Procedure

### Step 1 — Determine scope

Ask (or infer from context):
- Does the port apply to **V2 only**, **V3 only**, or **both**?
- Will the port data type be **auto-generated** (`"generated": true`) or **user-defined** (`"generated": false`)?
  - Use `generated: true` when the type is a plain struct with only primitive fields (float, uint*, int*).
  - Use `generated: false` when the type wraps a non-trivial C++ class or needs custom serialization logic.
- Should the port be recorded by default in the debug log (`record_by_default`)?
- How is the port written: periodically (at what rate, by which task/runnable), on events, or only once (e.g. at init)? This decides the invalidation timeout (Step 3a). If it is unclear, ask.

### Step 2 — Design the port data type

**The default-constructed value is the "invalid" value.** Auto-invalidation writes `T{}` to the port, so consumers must be able to recognize it as invalid. Prefer a validity indicator whose default is "not valid", e.g. `uValid_ = 0`, `eHealth_ = eUnavailable (0)`, or `uTimestampUs_ = 0`. If the type has no such field and zeros are also legitimate data (e.g. a plain `float`), invalidation is silent to consumers: add a validity field, or ask the user whether to disable invalidation for this port.

#### 2a. Auto-generated type (`"generated": true`)

Add a new entry to the `"port_data_types"` array in the relevant `rte.json`. Supported field types: `char`, `int8_t`, `uint8_t`, `int16_t`, `uint16_t`, `int32_t`, `uint32_t`, `int64_t`, `uint64_t`, `float`, `double`.

```json
{
  "name": "MyNewType",
  "fields": [
    { "name": "MyValue",      "type": "float",    "length": 1, "default_value": 0.0, "description": "..." },
    { "name": "TimestampUs",  "type": "uint64_t", "length": 1, "default_value": 0,   "description": "Timestamp in microseconds" },
    { "name": "Valid",        "type": "uint8_t",  "length": 1, "default_value": 0,   "description": "Validity flag" }
  ]
}
```

The generator will produce the struct in `RteTypesGenerated.h` with `ToByteVector()` / `FromByteVector()` implementations. The `default_value` of each field defines the value written on invalidation.

#### 2b. User-defined type (`"generated": false`)

Add the struct manually to `RteTypesUserDefined.h` in each affected variant. The default member initializers define the value written on invalidation; the type must be default-constructible. Template:

```cpp
struct SMyNewType final : public NLibCommon::CSerializable
{
  float fMyValue_{ 0.0F };       ///< Description [unit]
  uint64_t uTimestampUs_{ 0U };  ///< Timestamp in microseconds
  uint8_t uValid_{ 0U };         ///< Validity flag

#if defined(SEND_DEBUG_OUTPUT) || defined(_MSC_VER)
  std::vector<uint8_t> ToByteVector() final
  {
    std::vector<uint8_t> oBytes;
    bool bStatus = NLibCommon::Serialize(oBytes,
      fMyValue_,
      uTimestampUs_,
      uValid_);
    if (false == bStatus) { oBytes.clear(); }
    return oBytes;
  }

  bool FromByteVector(const std::vector<uint8_t>& korBytes) final
  {
    bool bStatus = NLibCommon::Deserialize(korBytes,
      fMyValue_,
      uTimestampUs_,
      uValid_);
    if (false == bStatus) { *this = SMyNewType(); }
    return bStatus;
  }
#endif // SEND_DEBUG_OUTPUT || _MSC_VER
};
```

**Field ordering in `ToByteVector` / `FromByteVector` must exactly match.** The byte stream is position-sensitive.

### Step 3 — Add the port entry to rte.json

Append the new port entry at the **end** of the `"ports"` array — after the last existing port, before the closing `]`. Never insert in the middle.

```json
{
  "name": "MyNewPort",
  "type": "MyNewType",
  "generated": true,
  "record_by_default": false
}
```

Optional key `"invalidation_timeout_ms"` (see Step 3a), e.g. for an event-driven port:

```json
{
  "name": "MyEventPort",
  "type": "MyEventType",
  "generated": true,
  "record_by_default": true,
  "invalidation_timeout_ms": 0
}
```

For `"generated": false`, the `"type"` string is the C++ class or struct name as it appears in the header (e.g., `"SMyNewType"`).

Repeat this step for each hardware variant that needs the port.

### Step 3a — Choose the invalidation timeout

**How the value is resolved (generator):**

| Source | Effect |
|---|---|
| Port key `"invalidation_timeout_ms"` | Used for this port if present |
| Top-level `"default_port_invalidation_timeout_ms"` | Used if the port key is absent (currently `1000` in V2 and V3) |
| Neither present | `0` |

- `0` disables invalidation for the port.
- Must be a JSON integer (not a bool, float, or string), `0`–`4294967295`. Otherwise the generator exits with `Error: invalidation_timeout_ms for port <name> must be ...` before writing any files.
- Emitted as the third template argument of the port in `CRte.h`: `CSoftwareComponentPort<SMyNewType, static_cast<uint8_t>(EPortIds::eMyNewPort), 1000U>`. With `0U` the check is compiled out (`if constexpr`).

**Runtime behavior (`CSoftwareComponentPort.h`):**
- No timer: the check runs lazily in `Read()` and `CPortReader::ReadPorts()`, under the port mutex. `Write()` only stores data and the write timestamp.
- If `now - lastWrite > timeout` (strictly greater; at exactly the timeout the data is kept), the port data is set to `T{}` and the reader receives that default value.
- An invalidation restarts the timeout, so a stale port is invalidated once per timeout period, not on every read. The next `Write()` restores normal data immediately.
- Before the first write the last-write timestamp is 0, so a never-written port is "invalidated" (to its already-default value) once `timeout` has elapsed since boot.
- In `SEND_DEBUG_OUTPUT` builds, an invalidation of a recorded port is logged as a regular port write event (default data, invalidation timestamp). Replay reproduces it with no tool changes; on the PC `GetMicroseconds()` returns the replay time.
- The timeout is not serialized: changing it does not change port IDs or the log format, so existing `.bin` logs stay replayable. It does change runtime behavior, so regenerate and rebuild.

**Decision rules:**

| Writer pattern | Setting |
|---|---|
| Periodic, interval well below 1 s (5 ms / 10 ms tasks, 25 Hz baro, sensor inputs, filter/monitor outputs) | Omit the key to inherit the global default |
| Periodic, worst-case interval (including jitter and startup gaps) close to or above 1 s | Explicit value comfortably above the worst-case interval; confirm it with the user |
| Event-driven or sporadic (e.g. `SyncPulseTime`), write-once at init (configuration, calibration), command/request ports | `"invalidation_timeout_ms": 0` |
| Faster stale-data detection required by a consumer (safety monitoring) | Explicit smaller value, still above the worst-case write interval; confirm it with the user |

- Omit the key rather than repeating `1000`, so the port follows future changes of the global default.
- Use the same value in V2 and V3 unless the hardware variants write the port differently.
- Changing `default_port_invalidation_timeout_ms` affects every port without an override in that variant. Ask the user before changing it, and check the ports that rely on it.
- When adding invalidation to an existing port, check every consumer: each must handle the default value (e.g. treat it as unavailable) rather than processing it as real data.

### Step 4 — Run code generation

For V2 (run from the Rte folder or use the absolute path):
```
02_Projects\Stm32Bahrs\TargetApp\Rte\generate_rte_bahrs_v2.bat
```

For V3:
```
02_Projects\Stm32Bahrs\TargetApp_HW_V3\bahrs-v3-rte\generate_rte_bahrs_v3.bat
```

Both scripts call `06_Tools/rte_generator/rte_generator/rte_generator.py` and regenerate:
- `CRte.h` / `CRte.cpp` (RTE class with new port member, its invalidation timeout template argument, and `EPortIds` entry)
- `RteTypesGenerated.h` (if `generated: true`)
- `WriteToRtePort.cpp` (PC-side helper)
- `DebugOutputToMat.cpp` (MAT exporter)

### Step 5 — Verify

1. **Check the generated declaration** in `CRte.h` (`TargetApp/Rte/rte_generated/Inc/` for V2, `TargetApp_HW_V3/bahrs-v3-rte/rte_generated/Inc/` for V3): the third template argument of the port equals the intended timeout.
2. **Build PC tools**: `05_Jenkins/Build_VS.bat Debug`
3. **Build embedded target**: `05_Jenkins/Build_TargetApp_STM32_V2.bat debug` (V2) or `05_Jenkins/Build_TargetApp_STM32_V3.bat debug` (V3)
4. **Unit tests** (if `CSoftwareComponentPort.h` or the generator changed): `05_Jenkins/Unit_Tests.bat Debug`

## Common Mistakes

- Inserting a port in the middle of the `"ports"` array → breaks replay tool binary compatibility.
- Using `bool` in a port data type → size mismatch between targets; use `uint8_t` instead.
- Adding a user-defined type but forgetting to update `RteTypesUserDefined.h` in the other hardware variant's folder.
- Field order mismatch between `ToByteVector` and `FromByteVector`.
- Running the generator but not rebuilding the VS solution → stale `DebugOutputToMat.cpp` in the build cache.
- Adding an event-driven, slow, or write-once port without `"invalidation_timeout_ms": 0` → it silently resets to default data 1 s after the last write.
- A port data type whose default value looks like valid data → consumers cannot detect an invalidated (stale) port.
- Writing `"invalidation_timeout_ms": "500"`, `500.0`, `true`, or a negative value → the generator rejects it; use a plain integer.
- Changing `default_port_invalidation_timeout_ms` to fix one port → affects all ports without an override; use the per-port key instead.
