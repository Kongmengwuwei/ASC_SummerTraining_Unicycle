@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" py -3 -m venv .venv
".venv\Scripts\python.exe" -m pip install -e ".[dev]"
if errorlevel 1 exit /b 1
".venv\Scripts\python.exe" -m pytest -q
if errorlevel 1 exit /b 1
".venv\Scripts\python.exe" scripts\build_windows.py
if errorlevel 1 exit /b 1
echo Ready: dist\EmbeddedStation\EmbeddedStation.exe
