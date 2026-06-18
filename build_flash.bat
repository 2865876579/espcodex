@echo off
setlocal EnableExtensions

set "PROJ=%~dp0"
set "PROJ=%PROJ:~0,-1%"
cd /d "%PROJ%"

set "ESP_PORT=COM3"
set "ESP_BAUD=460800"
if not "%~1"=="" set "ESP_PORT=%~1"

set "IDF_TOOLS=D:\Espressif"
set "PYTHON=%IDF_TOOLS%\python_env\idf5.1_py3.11_env\Scripts\python.exe"
set "ESPTOOL=%PYTHON% %IDF_TOOLS%\frameworks\esp-idf-v5.1.2\components\esptool_py\esptool\esptool.py"
set "NINJA=%IDF_TOOLS%\tools\ninja\1.10.2\ninja.exe"

echo ============================================
echo   ESP32-S3 Build ^& Flash (AEC enabled)
echo   Port: %ESP_PORT%
echo ============================================
echo.

rem ---------- Step 1: Build ----------
echo [1/3] Building firmware...
"%NINJA%" -C "%PROJ%\build"
if errorlevel 1 (
    echo BUILD FAILED
    pause
    exit /b 1
)
echo.

rem ---------- Step 2: Generate srmodels.bin ----------
echo [2/3] Packing wake word model...
if not exist "%PROJ%\build\srmodels" mkdir "%PROJ%\build\srmodels"
"%PYTHON%" "%PROJ%\managed_components\espressif__esp-sr\model\movemodel.py" -d1 "%PROJ%\sdkconfig" -d2 "%PROJ%\managed_components\espressif__esp-sr" -d3 "%PROJ%\build"
if errorlevel 1 (
    echo MODEL PACK FAILED
    pause
    exit /b 1
)
echo.

rem ---------- Step 3: Flash ----------
echo [3/3] Flashing to %ESP_PORT% at %ESP_BAUD% baud...
"%PYTHON%" "%IDF_TOOLS%\frameworks\esp-idf-v5.1.2\components\esptool_py\esptool\esptool.py" -p %ESP_PORT% -b %ESP_BAUD% --before default_reset --after hard_reset --chip esp32s3 write_flash --flash_mode dio --flash_freq 80m --flash_size 16MB 0x0 "%PROJ%\build\bootloader\bootloader.bin" 0x8000 "%PROJ%\build\partition_table\partition-table.bin" 0x10000 "%PROJ%\build\pump_control.bin" 0x510000 "%PROJ%\build\srmodels\srmodels.bin"
if errorlevel 1 goto flash_fail

echo.
echo ============================================
echo   Done. ESP32 restarting...
echo ============================================
pause
exit /b 0

:flash_fail
echo.
echo FLASH FAILED - Is %ESP_PORT% busy?
echo Close serial monitor and retry.
pause
exit /b 1
