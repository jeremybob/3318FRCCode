@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "ROOT_DIR=%~dp0"
cd /d "%ROOT_DIR%"

if defined DASHBOARD_JAVA_HOME (
  if exist "%DASHBOARD_JAVA_HOME%\bin\java.exe" set "JAVA_HOME=%DASHBOARD_JAVA_HOME%"
)

if defined JAVA_HOME (
  if exist "%JAVA_HOME%\bin\java.exe" set "PATH=%JAVA_HOME%\bin;%PATH%"
)

where java >nul 2>nul
if not errorlevel 1 goto java_ok

call :discover_wpilib_java "%USERPROFILE%\wpilib"
if not defined JAVA_HOME call :discover_wpilib_java "%PUBLIC%\wpilib"

if defined JAVA_HOME (
  set "PATH=%JAVA_HOME%\bin;%PATH%"
)

where java >nul 2>nul
if errorlevel 1 (
  echo No Java runtime found.
  echo Install WPILib ^(includes Java^) or set DASHBOARD_JAVA_HOME/JAVA_HOME.
  echo Searched under: %USERPROFILE%\wpilib and %PUBLIC%\wpilib
  exit /b 1
)

:java_ok
set "DASH_ARGS=--team 3318"
if not "%~1"=="" (
  set "DASH_ARGS=%*"
)

call gradlew.bat runDashboard --args="%DASH_ARGS%"
exit /b %ERRORLEVEL%

:discover_wpilib_java
set "BASE_DIR=%~1"
if not exist "%BASE_DIR%" exit /b 0

for /f "delims=" %%Y in ('dir /b /ad "%BASE_DIR%" 2^>nul') do (
  set "YEAR_DIR=%BASE_DIR%\%%Y"
  if exist "!YEAR_DIR!\jdk\bin\java.exe" (
    set "JAVA_HOME=!YEAR_DIR!\jdk"
  )
  for /f "delims=" %%J in ('dir /b /ad "!YEAR_DIR!\jdk-*" 2^>nul') do (
    if exist "!YEAR_DIR!\%%J\bin\java.exe" (
      set "JAVA_HOME=!YEAR_DIR!\%%J"
    )
  )
)
exit /b 0
