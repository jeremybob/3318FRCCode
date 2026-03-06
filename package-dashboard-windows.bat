@echo off
setlocal EnableExtensions

set "ROOT_DIR=%~dp0"
cd /d "%ROOT_DIR%"

set "TASK=packageDashboardWindows"
set "TASK_LABEL=Windows app image"
set "EXIT_CODE=0"
set "FORWARD_ARGS=%*"

if /I "%~1"=="installer" goto installer
if /I "%~1"=="--installer" goto installer
if /I "%~1"=="help" goto usage
if /I "%~1"=="--help" goto usage
goto run

:installer
set "TASK=packageDashboardWindowsInstaller"
set "TASK_LABEL=Windows installer"
shift
set "FORWARD_ARGS=%*"

:run

echo Packaging %TASK_LABEL% for 3318 Dashboard...
echo.
call gradlew.bat %TASK% %FORWARD_ARGS%
set "EXIT_CODE=%ERRORLEVEL%"
echo.

if not "%EXIT_CODE%"=="0" (
  echo Packaging failed with exit code %EXIT_CODE%.
  echo.
  echo Requirements:
  echo   - Run this on Windows.
  echo   - Use a full JDK 17.
  echo   - For installer builds, install WiX first.
  goto end
)

if /I "%TASK%"=="packageDashboardWindowsInstaller" (
  echo Installer created under:
  echo   build\dashboard-package\windows\installer
) else (
  echo App image created under:
  echo   build\dashboard-package\windows\image
)

goto end

:usage
echo Usage:
echo   package-dashboard-windows.bat
echo   package-dashboard-windows.bat installer
echo.
echo Default:
echo   Builds the standalone Windows app image.
echo.
echo installer:
echo   Builds the installable Windows EXE ^(requires WiX^).

:end
echo.
pause
exit /b %EXIT_CODE%
