@echo off
setlocal EnableExtensions

rem Build and flash the ESP-IDF firmware. Usage: build_flash.bat [COM_PORT] [--nopause]
set "PROJECT_DIR=%~dp0"
if "%PROJECT_DIR:~-1%"=="\" set "PROJECT_DIR=%PROJECT_DIR:~0,-1%"

set "ESP_PORT=COM3"
set "NO_PAUSE=0"
if not "%~1"=="" if /i not "%~1"=="--nopause" set "ESP_PORT=%~1"
if /i "%~1"=="--nopause" set "NO_PAUSE=1"
if /i "%~2"=="--nopause" set "NO_PAUSE=1"

set "IDF_PATH=D:\Espressif\frameworks\esp-idf-v5.1.2"
set "IDF_TOOLS_PATH=D:\Espressif"
set "IDF_PYTHON=D:\Espressif\python_env\idf5.1_py3.11_env\Scripts\python.exe"
set "IDF_PY=%IDF_PATH%\tools\idf.py"
set "PYTHONUTF8=1"
set "PYTHONIOENCODING=utf-8"
set "PATH=D:\Espressif\tools\xtensa-esp32s3-elf\esp-12.2.0_20230208\xtensa-esp32s3-elf\bin;D:\Espressif\tools\cmake\3.24.0\bin;D:\Espressif\tools\ninja\1.11.0;%PATH%"

if not exist "%IDF_PYTHON%" (
  echo Python not found: %IDF_PYTHON%
  goto fail
)
if not exist "%IDF_PY%" (
  echo idf.py not found: %IDF_PY%
  goto fail
)

echo ==============================
echo ESP-IDF build + flash
echo Project: %PROJECT_DIR%
echo Port:    %ESP_PORT%
echo ==============================
echo.

echo [1/2] Building firmware...
"%IDF_PYTHON%" "%IDF_PY%" -C "%PROJECT_DIR%" build
if errorlevel 1 goto fail

echo.
echo [2/2] Flashing firmware to %ESP_PORT%...
"%IDF_PYTHON%" "%IDF_PY%" -C "%PROJECT_DIR%" -p "%ESP_PORT%" flash
if errorlevel 1 goto fail

echo.
echo Done.
if "%NO_PAUSE%" neq "1" pause
exit /b 0

:fail
set "ERR=%errorlevel%"
if "%ERR%"=="0" set "ERR=1"
echo.
echo Failed with code %ERR%.
echo If %ESP_PORT% is busy, close the serial monitor and rerun this script.
if "%NO_PAUSE%" neq "1" pause
exit /b %ERR%