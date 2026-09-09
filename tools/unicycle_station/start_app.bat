@echo off
setlocal
cd /d "%~dp0"
if not exist "dist\EmbeddedStation\EmbeddedStation.exe" (
  echo Application not built. Run build_windows.bat first.
  pause
  exit /b 1
)
start "" "dist\EmbeddedStation\EmbeddedStation.exe"
