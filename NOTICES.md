# Third-Party Notices

This repository includes third-party software. Each component stays under its
own license, so keep the license and copyright notices when redistributing it.

## Bundled components

- **Eigen 3.4.0**: MPL-2.0 (a few files use BSD, LGPL, GPL, Apache or MINPACK
  terms). See `99_3rd_party/eigen-3.4.0/COPYING.*`.
- **Open Aided Navigation**: MIT. See `99_3rd_party/open-aided-navigation/LICENSE.txt`.
- **TinyMAT**: LGPL-2.1-or-later. See `99_3rd_party/TinyMAT/LICENSE`.
- **GoogleTest 1.12.1**: BSD-3-Clause. See `99_3rd_party/googletest-release-1.12.1/LICENSE`.
- **libcanard (DroneCAN)**: MIT. See `99_3rd_party/libcanard/LICENSE`.
- **MathJax 3.2.2**: Apache-2.0. See `99_3rd_party/mathjax/LICENSE`.
- **CANAerospace**: MIT. See
  `02_Projects/Stm32Bahrs/shared/bsw/swc-can-aero/CANaerospace/LICENSE`.
- **FreeRTOS**: MIT. See `LICENSE` under `Middlewares/Third_Party/FreeRTOS/Source/`.
- **CMSIS and STM32F4xx device files**: Apache-2.0. See `LICENSE.txt` under
  `Drivers/CMSIS/`.
- **STM32F4xx HAL driver**: BSD-3-Clause. See `LICENSE.txt` under
  `Drivers/STM32F4xx_HAL_Driver/`.

The FreeRTOS, CMSIS and HAL files are part of the STM32Cube projects in
`02_Projects/Stm32Bahrs/` (`TargetApp/bahrs-v2-cube` and `TargetApp_HW_V3/bahrs-v3-cube`).

## Precompiled PC tool dependencies

- **Qt 5.12.12 Core** (`Qt5Core.dll`, `Qt5Cored.dll`): LGPL-3.0, used by
  TinyMAT. The source of this Qt version is available at
  <https://download.qt.io/archive/qt/5.12/5.12.12/>. The DLLs are
  shipped next to the precompiled tools in
  `02_Projects/VisualStudioToolsBahrs/WorkDir/` and the `Tools`/`Utilities` folders
  under `03_Firmware/BahrsTargetApp/`.

## Documentation tools

- **PlantUML 1.2023.1** (`99_3rd_party/plantuml/`): GPL-3.0-or-later.
- **Doxygen 1.17.0** (`99_3rd_party/doxygen/`): GPL-2.0. It ships with
  `libclang.dll` (LLVM, Apache-2.0 with LLVM exceptions).

## Bosch Sensortec-derived drivers

The BMM150 driver (`02_Projects/Stm32Bahrs/TargetApp/bsw/bmm150-driver/`) and the
BMM350 driver (`02_Projects/Stm32Bahrs/TargetApp_HW_V3/bsw/swc-bmm350-driver/`)
are heavily refactored versions of code from Bosch Sensortec GmbH.

- BMM150: Copyright (c) 2020 Bosch Sensortec GmbH
- BMM350: Copyright (c) 2024 Bosch Sensortec GmbH

BSD-3-Clause license:

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
