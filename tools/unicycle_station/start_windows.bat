@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  py -3 -m venv .venv
  if errorlevel 1 goto fail
)
".venv\Scripts\python.exe" -c "import PySide6, pyqtgraph, serial, OpenGL" >nul 2>&1
if errorlevel 1 (
  ".venv\Scripts\python.exe" -m pip install -e ".[dev]"
  if errorlevel 1 goto fail
)
".venv\Scripts\python.exe" run_station.py %*
if errorlevel 1 goto fail
exit /b 0
:fail
echo Startup failed. Check Python 3.11+ and the output above.
pause
exit /b 1
