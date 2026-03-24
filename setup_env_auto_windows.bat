@echo off
setlocal EnableExtensions EnableDelayedExpansion

REM Auto bootstrap script for Windows.
REM
REM What it does:
REM   1) Finds the project root (this folder)
REM   2) Creates a Python virtual environment in .\venv
REM   3) Upgrades pip/wheel/setuptools
REM   4) Installs dependencies from requirements.txt
REM
REM Usage:
REM   Double-click this file, or run it from CMD:
REM     setup_env_auto_windows.bat
REM
REM Notes:
REM   - Prefers the Windows Python Launcher: py -3
REM   - Falls back to python if py is unavailable
REM   - If venv already exists, it will be moved to a timestamped backup folder

set "SCRIPT_DIR=%~dp0"
set "PROJECT_DIR=%SCRIPT_DIR:~0,-1%"
set "VENV_DIR=%PROJECT_DIR%\venv"
set "REQ_FILE=%PROJECT_DIR%\requirements.txt"

echo [Info] Project dir: %PROJECT_DIR%
echo [Info] Requirements: %REQ_FILE%
echo.

if not exist "%REQ_FILE%" (
  echo [ERROR] requirements.txt not found.
  exit /b 1
)

set "PYTHON_CMD="
where py >nul 2>nul
if %errorlevel% equ 0 (
  set "PYTHON_CMD=py -3"
) else (
  where python >nul 2>nul
  if %errorlevel% equ 0 (
    set "PYTHON_CMD=python"
  )
)

if not defined PYTHON_CMD (
  echo [ERROR] Python not found. Please install Python 3 and make sure it is added to PATH.
  exit /b 1
)

echo [Info] Using Python command: %PYTHON_CMD%
echo.

if exist "%VENV_DIR%" (
  for /f %%i in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd_HHmmss"') do set "TS=%%i"
  set "BACKUP_DIR=%PROJECT_DIR%\venv.bak_!TS!"
  echo [Info] Existing venv found, moving to: !BACKUP_DIR!
  move "%VENV_DIR%" "!BACKUP_DIR!" >nul
)

echo [Info] Creating virtual environment...
%PYTHON_CMD% -m venv "%VENV_DIR%"
if errorlevel 1 (
  echo [ERROR] Failed to create virtual environment.
  exit /b 1
)

call "%VENV_DIR%\Scripts\activate.bat"
if errorlevel 1 (
  echo [ERROR] Failed to activate virtual environment.
  exit /b 1
)

echo [Info] Upgrading pip, wheel, setuptools...
python -m pip install --upgrade pip wheel setuptools
if errorlevel 1 (
  echo [ERROR] Failed to upgrade pip tools.
  exit /b 1
)

echo [Info] Installing dependencies from requirements.txt...
python -m pip install -r "%REQ_FILE%"
if errorlevel 1 (
  echo [ERROR] Failed to install dependencies.
  exit /b 1
)

echo.
echo [OK] Environment created successfully.
echo [OK] Next time, activate it with:
echo      %VENV_DIR%\Scripts\activate.bat
echo [OK] Or run Python directly from:
echo      %VENV_DIR%\Scripts\python.exe

endlocal
exit /b 0