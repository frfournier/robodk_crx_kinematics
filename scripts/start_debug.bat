@echo off
setlocal EnableExtensions

set "CRXKIN_VERBOSE=1"
set "CRXKIN_LOG=%TEMP%\crxkinematics.debug.log"

rem Repo root assumed as parent of /scripts
set "REPO_ROOT=%~dp0.."
for %%I in ("%REPO_ROOT%") do set "REPO_ROOT=%%~fI"

set "ROBOT_REL=assets\Fanuc-CRX-10iA-Custom.robot"
set "ROBOT=%REPO_ROOT%\%ROBOT_REL%"

if not exist "%ROBOT%" (
  echo [ERROR] Robot file not found: "%ROBOT%"
  exit /b 2
)

start "" "C:\Program Files\RoboDK\bin\RoboDK.exe" "%ROBOT%"
endlocal