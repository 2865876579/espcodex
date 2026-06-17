@echo off
setlocal

if /i "%~1" neq "--run" (
  start "ESPS3 Auto Push" cmd /k ""%~f0" --run"
  exit /b
)

set "NO_PAUSE=0"
if /i "%~2"=="--nopause" set "NO_PAUSE=1"

cd /d "%~dp0"

echo ==============================
echo espcodex auto push
echo Folder: %CD%
echo ==============================
echo.

for /f "usebackq delims=" %%i in (`powershell -NoProfile -Command "Get-Date -Format 'yyyy-MM-dd HH:mm:ss'"`) do set "NOW=%%i"

git add -A

rem Keep local secrets and tool state out of automatic commits.
git restore --staged -- .env .env.local .claude/settings.local.json 2>nul

git diff --cached --quiet
if %errorlevel% equ 0 (
  echo No staged changes to commit.
  git status --short
  if "%NO_PAUSE%" neq "1" pause
  exit /b 0
)

git commit -m "auto backup %NOW%"
if errorlevel 1 (
  echo Commit failed.
  if "%NO_PAUSE%" neq "1" pause
  exit /b 1
)

git push
if errorlevel 1 (
  echo Push failed.
  if "%NO_PAUSE%" neq "1" pause
  exit /b 1
)

echo Done.
if "%NO_PAUSE%" neq "1" pause
