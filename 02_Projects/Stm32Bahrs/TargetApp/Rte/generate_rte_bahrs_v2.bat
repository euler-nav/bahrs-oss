@echo off
REM @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.
REM Batch script to run RTE generator

python ../../../../06_Tools/rte_generator/rte_generator/rte_generator.py rte.json ^
  --rte-path rte_generated/ ^
  --debug-output-path ../../../VisualStudioToolsBahrs/DebugOutputToMatBahrsV2 ^
  --replay-tool-path ../../../VisualStudioToolsBahrs/BahrsReplayToolBahrsV2/

pause
