@echo off
setlocal
cd /d "%~dp0"

rem Prefer the project's verified Conda environment.
set "IMU_CLIB_PYTHON=C:\Users\14943\.conda\envs\imu_clib\python.exe"
if exist "%IMU_CLIB_PYTHON%" (
    "%IMU_CLIB_PYTHON%" imu_calibration\quickcal_station_main.py
    goto :done
)

set "IMU_CLIB_CONDA=D:\Jay\software_APP\miniconda\Scripts\conda.exe"
if exist "%IMU_CLIB_CONDA%" (
    "%IMU_CLIB_CONDA%" run -n imu_clib python imu_calibration\quickcal_station_main.py
    goto :done
)

where python >nul 2>nul
if not errorlevel 1 (
    python imu_calibration\quickcal_station_main.py
    goto :done
)
where py >nul 2>nul
if not errorlevel 1 (
    py -3 imu_calibration\quickcal_station_main.py
    goto :done
)
echo [ERROR] Conda environment "imu_clib" was not found and no fallback Python is available.
echo Expected interpreter: %IMU_CLIB_PYTHON%
:done
if errorlevel 1 pause
endlocal
