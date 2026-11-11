# EULER-NAV BAHRS software

This repository contains software for the [EULER-NAV BAHRS](https://euler-nav.com/bahrs), a baro-inertial attitude and heading reference system. The device provides attitude, height, vertical speed, angular rates and specific force over RS-232 and CAN, using redundant inertial, pressure and magnetic sensors.

![EULER-NAV BAHRS](00_Documentation/img/BAHRS_main_image.jpg)

The repository includes firmware for BAHRS hardware revisions 2 and 3, a PC replay tool, log evaluation and calibration scripts, configuration tools and software architecture documentation. The sensor fusion algorithm is supplied as precompiled libraries. Source code of the sensor fusion library is planned for publication
mid December 2026.

## Get the repository

The repository and its tools target **Windows 10/11**. Install [Git for Windows](https://git-scm.com/download/win), which includes Git LFS. Then, in PowerShell or Command Prompt, clone to a short path such as `C:\Work\bahrs-oss`:

```bat
git config --global core.longpaths true
git lfs install
git clone --recurse-submodules https://github.com/euler-nav/bahrs-oss.git C:\Work\bahrs-oss
cd C:\Work\bahrs-oss
git lfs pull
```

The short path and `core.longpaths` avoid "file name too long" errors in the deeply nested STM32Cube and third-party folders.

Two third-party dependencies are Git submodules: `99_3rd_party\open-aided-navigation` and `99_3rd_party\libcanard`. Both must be reachable when cloning. If you cloned without `--recurse-submodules`, run `git submodule update --init --recursive`. After switching branches or pulling changes, run it again to check out the required submodule revisions.

Git LFS normally downloads the binary files during checkout; `git lfs pull` also retrieves them if they were skipped. These include libraries required for builds, precompiled PC tools and documentation assets. Run `git lfs pull` if LFS files are missing or still contain pointer text.

The build and replay instructions below describe the Windows workflow.

## Build firmware

- Toolchain installation (STM32CubeIDE 1.18.0 with the suggested toolchain, CMake, Ninja), command-line and STM32CubeIDE builds: see the [BAHRS rev. 3 project README](02_Projects/Stm32Bahrs/TargetApp_HW_V3/README.md). The rev. 2 specifics are in the [BAHRS rev. 2 project README](02_Projects/Stm32Bahrs/TargetApp/README.md).
- Quick start from the repository root: `05_Jenkins\Build_TargetApp_STM32_V3.bat release` (or `..._V2.bat` for BAHRS rev. 2). The output is placed in `build\release` of the corresponding project.
- The variant argument is required and lowercase: `debug`, `release` or `debug-with-debug-output`. The script deletes and recreates the build folder of that variant.
- The `debug-with-debug-output` variant produces the extended debug logs needed by the replay and calibration tools. In this variant the RS-232 output runs at 921600 baud instead of 115200.

## Flash a device

Devices ship with the bootloader at address 0x8000000. After power-on it waits 0.5 s for STM32CubeProgrammer to connect over RS-232, then starts the application at 0x8008000. To replace the application:

- Flash the `.hex` file, **never the `.elf`**. Programming an ELF file erases the whole flash, including the bootloader.
- Click *Connect* in STM32CubeProgrammer within 0.5 s after powering the device.

The [knowledge base](00_Documentation/KNOWLEDGE_BASE.md) has a step-by-step guide with screenshots and the debug connector pinout. Prebuilt images, and which one fits which hardware revision, are listed in [03_Firmware](03_Firmware/README.md). If the bootloader of a rev. 2 device was erased, program `03_Firmware\BahrsBootloader_V_1_0.hex` with an ST-LINK through the debug connector (SWD). For rev. 3, contact us via *info@euler-nav.com*.

## Replay and evaluate recordings

The replay tool re-runs application software components against recorded port-write and runnable-call events, allowing a flight or other recording to be re-simulated and internal results plotted. Replay requires an **extended debug-output** binary log from a compatible firmware and hardware revision; a normal RS-232 output log alone is not a replay input.

### Try the bundled demo

[BahrsEvaluationTool](02_Projects/BahrsEvaluationTool/README.md) includes two sample recordings of BAHRS rev. 3, and the replay executables are already checked in under `02_Projects\VisualStudioToolsBahrs\WorkDir`. In Octave, set the current directory to `02_Projects\BahrsEvaluationTool` and run `script_runBahrsFilterReplay.m`. This is the quickest way to explore the replay without building the Visual Studio solution.

The MAT converter (via the bundled TinyMAT and Qt DLLs) needs the [Microsoft Visual C++ Redistributable for Visual Studio 2015-2022 (x64)](https://aka.ms/vs/17/release/vc_redist.x64.exe). It is already present if Visual Studio is installed.

The PC replay build does not read the calibration from the device NVM. The magnetometer and pressure calibration values are hard-coded in `CMagnetometerCompensator.cpp` and `CPressureCompensator.cpp`, and they fit the two sample recordings only. For your own recordings, replace them with the output of [mag_calib.m](06_Tools/MagCalib/mag_calib.m) (see below) and rebuild the replay tool.

### Build and run the replay tools

1. Install [Visual Studio Community 2022](https://visualstudio.microsoft.com/vs/community/) with the **Desktop development with C++** workload.
2. Open [VisualStudioToolsBahrs.sln](02_Projects/VisualStudioToolsBahrs/VisualStudioToolsBahrs.sln), select `Debug` or `Release` and `x64`, and build the solution. The replay and MAT-conversion executables are placed in `02_Projects\VisualStudioToolsBahrs\WorkDir`; the required runtime DLLs are provided there through Git LFS. Building overwrites the corresponding checked-in executables.
3. Use GNU Octave 11.3.0, which we use for these scripts. They are also expected to work with MATLAB.
4. Set Octave's current directory to `02_Projects\BahrsEvaluationTool`. In [script_runBahrsFilterReplay.m](02_Projects/BahrsEvaluationTool/script_runBahrsFilterReplay.m), set `testConfig.binaryFile` to your debug log, `testConfig.outputBinaryFile` to the desired replay output, and `testConfig.bahrsVersion` to `V2` (BAHRS rev. 2) or `V3` (BAHRS rev. 3) to match the recording. The checked-in `binaryFile` is the first sample recording (rev. 3, `V3`). Choose the plots with the `testConfig.plot...` flags, then run the script.

The script runs the matching replay executable, converts its output to a MAT file and plots selected ports. A relative output binary path is resolved from the current directory; the MAT file is written there using the binary file's name. Adjust the executable paths in the script if the tools are located elsewhere.

For ordinary RS-232 output and CAN trace analysis rather than replay, configure and run [script_evaluateBahrs.m](02_Projects/BahrsEvaluationTool/script_evaluateBahrs.m). It uses `SerialProtocolToMat.exe` to convert a serial log, or can load an existing MAT file; a CAN trace can be evaluated separately.

## Calibrate sensors

[mag_calib.m](06_Tools/MagCalib/mag_calib.m) estimates hard- and soft-iron parameters for the three magnetometers and, optionally, relative pressure offsets for the barometers. It accepts a compatible extended debug-output `.bin` recording or a converted `.mat` file.

Set Octave's current directory to `06_Tools\MagCalib` and edit the configuration block at the top of the script:

- `config.inputPath` - the recording. It defaults to the bundled rev. 3 sample `20260824_mag_calibration_log.bin`, so the script runs as shipped.
- `config.referenceMagneticFieldNed` - the local Earth magnetic field in gauss.
- `config.converterExecutable` - used for binary input. It defaults to the rev. 3 converter in `WorkDir`; use `DebugOutputToMatBahrsV2.exe` for rev. 2 recordings.

The script saves a MAT results file and can export an NVM map with calibrated defaults, which can be opened in [NVMManager](06_Tools/NVMManager/README.md). The exported map uses the NVM format of the latest firmware built. It may differ from NVM formats of previous firmware releases.

## Repository guide

- [02_Projects/Stm32Bahrs](02_Projects/Stm32Bahrs/) - BAHRS rev. 2 and rev. 3 target applications, shared application components and libraries.
- [02_Projects/VisualStudioToolsBahrs](02_Projects/VisualStudioToolsBahrs/) - replay, MAT conversion and unit test projects.
- [02_Projects/BahrsEvaluationTool](02_Projects/BahrsEvaluationTool/) - Octave/MATLAB replay and log plotting scripts, with sample recordings.
- [03_Firmware](03_Firmware/README.md) - prebuilt firmware images and their usage notes.
- [05_Jenkins](05_Jenkins/) - firmware build and unit test scripts used above. `Jenkinsfile` and `Check_Submodules.bat` belong to our internal CI and are not needed for local builds.
- [06_Tools](06_Tools/) - calibration, NVM management, and RTE and NVM code generation tools.
- [00_Documentation](00_Documentation/README.md) - software architecture documentation, local generation instructions and a [knowledge base](00_Documentation/KNOWLEDGE_BASE.md). The architecture documentation is also [published online](https://euler-nav.github.io/bahrs-arch-docs/); product documentation and the serial protocol are on the [BAHRS documentation page](https://euler-nav.com/bahrsdoc).
- [99_3rd_party](99_3rd_party/) - bundled and submodule third-party dependencies.
- [.github](.github/) - instructions and skills for GitHub Copilot.

Python 3 on `PATH` is needed for the code generators (RTE, NVM driver); NVMManager additionally requires Python 3.11 and Pipenv, see its README. After changing `rte.json`, regenerate the RTE with the `generate_rte_bahrs_v2.bat` (rev. 2) or `generate_rte_bahrs_v3.bat` (rev. 3) script next to it. Run the script from its own folder or by double-clicking it, because it uses relative paths.

The Visual Studio solution includes the unit tests; after building it, run `05_Jenkins\Unit_Tests.bat Debug` or `Release` from the repository root. Unlike the firmware scripts, the Visual Studio configurations are capitalised.

## Support

Need help? Reach out to *info@euler-nav.com*.

## License

AMS-developed software, including the precompiled sensor fusion libraries, is distributed under the [3-Clause BSD License](LICENSE.txt). Third-party components have their own licenses; see [NOTICES.md](NOTICES.md) and the license files included with those components.
