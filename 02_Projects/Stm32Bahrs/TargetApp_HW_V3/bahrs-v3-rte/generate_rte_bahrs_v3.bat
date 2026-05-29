@echo off
REM Batch script to run RTE generator

python ../../../../06_Tools/rte_generator/rte_generator/rte_generator.py rte.json ^
  --rte-path .

pause
