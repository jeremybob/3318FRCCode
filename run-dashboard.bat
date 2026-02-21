@echo off
setlocal

set "ROOT_DIR=%~dp0"
cd /d "%ROOT_DIR%"

set "DASH_ARGS=--team 3318"
if not "%~1"=="" (
  set "DASH_ARGS=%*"
)

call gradlew.bat runDashboard --args="%DASH_ARGS%"
